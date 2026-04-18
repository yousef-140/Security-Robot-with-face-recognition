/*
 * =====================================================================
 * Security Robot — ESP32-CAM AI Thinker
 * Arduino IDE + ESP32 Arduino Core 1.0.6  ← REQUIRED VERSION
 * =====================================================================
 *
 * BOARD SETTINGS (Tools menu):
 *   Board            : AI Thinker ESP32-CAM
 *   Partition Scheme : Huge APP (3MB No OTA / 1MB SPIFFS)
 *   PSRAM            : Enabled
 *   CPU Frequency    : 240 MHz
 *   Flash Frequency  : 80 MHz
 *   Flash Mode       : QIO
 *   Upload Speed     : 115200
 *
 * WIRING:
 *   GPIO12 → L298N IN1      GPIO13 → L298N IN2
 *   GPIO14 → L298N IN3      GPIO15 → L298N IN4
 *   GPIO4  → Buzzer +
 *   GPIO2  → HC-SR04 TRIG   GPIO16 → HC-SR04 ECHO (voltage divider)
 *
 * FACE DETECTION:
 *   fd_forward.h is part of Core 1.0.6 — no external library needed.
 *   Checks for faces every FACE_CHECK_INTERVAL_MS milliseconds.
 *   Also available on demand via HTTP GET /face
 *   Buzzer beeps when a face is found.
 *
 * KNOWN BEHAVIOUR:
 *   Face detection takes ~600–900ms per call.
 *   During that time the MJPEG stream pauses briefly — this is normal.
 * =====================================================================
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "fd_forward.h"       //    Face detection — Core 1.0.6 ONLY
                              //    Provides: mtmn_config_t, box_array_t,
                              //              face_detect(), fmt2rgb888()

//  WiFi — edit these 
const char* ssid     = "****";
const char* password = "****";

//  Camera — AI Thinker pinout (do not change)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//  Motor pins 
#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 15

//  Buzzer 
#define BUZZER_PIN 4

//  HC-SR04 
#define TRIG_PIN 2
#define ECHO_PIN 16   // via voltage divider

//  Safety 
#define SAFE_DISTANCE_CM  20
#define MOVE_PULSE_MS     400

//  Face detection timing 
// How often the background loop runs a face check.
// 4000ms = every 4 seconds. Lower = more responsive, more CPU load.
#define FACE_CHECK_INTERVAL_MS  4000

//  Face detection config (global, set up in setup()) 
// These values are taken directly from Espressif's Core 1.0.6
// CameraWebServer example — the known-good baseline.
static mtmn_config_t mtmn_config;

static unsigned long lastFaceCheck = 0;

//  HTTP servers ─
httpd_handle_t control_httpd = NULL;
httpd_handle_t stream_httpd  = NULL;




//  MOTOR FUNCTIONS


void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(MOVE_PULSE_MS);
  stopMotors();
}

void moveBackward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(MOVE_PULSE_MS);
  stopMotors();
}

void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(MOVE_PULSE_MS);
  stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(MOVE_PULSE_MS);
  stopMotors();
}


//  ULTRASONIC

long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(4);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long d = pulseIn(ECHO_PIN, HIGH, 25000UL);
  return (d == 0) ? 999 : d / 58L;
}

bool pathIsClear() {
  long dist = getDistanceCM();
  Serial.printf("[US] Distance: %ld cm\n", dist);
  return dist > SAFE_DISTANCE_CM;
}
 

//  BUZZER


void beep(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(offMs);
  }
}

void startupBeep()  { beep(2, 100, 100); }  // 2 short  = boot OK
void connectedBeep(){ beep(1, 600,   0); }  // 1 long   = WiFi up
void warningBeep()  { beep(3,  80,  60); }  // 3 rapid  = obstacle
void errorBeep()    { beep(6, 200, 100); }  // 6 slow   = fatal error

// ★ FACE DETECTION BUZZER
void faceBeep()     { beep(3, 200, 100); }  // 3 medium = face detected!


// 

//  FACE DETECTION  ← this entire function is new
// 

//
//  How it works:
//    1. Capture a JPEG frame from the camera.
//    2. Allocate a PSRAM buffer for RGB888 (320×240×3 = ~230 KB).
//    3. Convert JPEG → RGB888 using fmt2rgb888() from fd_forward.h.
//    4. Run face_detect() — the MTMN neural network built into Core 1.0.6.
//    5. If box_array_t is non-null, at least one face was found.
//    6. Free everything, return true/false.
//
//  box_array_t fields in Core 1.0.6:
//    .box[]      — bounding boxes
//    .landmark[] — facial landmarks
//    .len        — number of faces found  (NOT .size — that field does not exist)
//
//  Memory:
//    RGB buffer lives in PSRAM (allocated by dl_matrix3du_alloc).
//    Core heap is not touched by the large buffer.
//    Always call dl_matrix3du_free() and free(net_boxes->...) to avoid leaks.

bool checkForFace() {
  // Capture frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[FACE] Frame capture failed");
    return false;
  }

  // Allocate RGB888 buffer in PSRAM
  // dl_matrix3du_alloc(n_items, width, height, channels)
  dl_matrix3du_t *rgb_buf = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!rgb_buf) {
    Serial.println("[FACE] RGB buffer alloc failed — PSRAM enabled?");
    esp_camera_fb_return(fb);
    return false;
  }

  // Convert JPEG frame to RGB888
  // fmt2rgb888 returns true on success
  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf->item);
  esp_camera_fb_return(fb);   // Release frame buffer immediately

  if (!converted) {
    Serial.println("[FACE] JPEG->RGB888 conversion failed");
    dl_matrix3du_free(rgb_buf);
    return false;
  }

  // Run the neural network
  box_array_t *net_boxes = face_detect(rgb_buf, &mtmn_config);
  dl_matrix3du_free(rgb_buf);   // Done with RGB buffer

  if (net_boxes) {
    // .len = number of faces found in Core 1.0.6 (not .size)
    Serial.printf("[FACE] Face(s) detected: %d\n", net_boxes->len);

    // Must free all parts of box_array_t to avoid memory leak
    free(net_boxes->box);
    if (net_boxes->landmark) free(net_boxes->landmark);
    free(net_boxes);
    return true;
  }

  Serial.println("[FACE] No face");
  return false;
}

//  HTML CONTROL PAGE

static const char PROGMEM PAGE_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Security Robot</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0d0d0d; color: #e8e8e8;
    font-family: 'Courier New', monospace;
    display: flex; flex-direction: column;
    align-items: center; padding: 16px; min-height: 100vh;
  }
  h2 { font-size: 22px; margin-bottom: 10px; color: #00e5ff; letter-spacing: 2px; }
  #stream-box {
    border: 2px solid #00e5ff33; border-radius: 8px;
    overflow: hidden; margin-bottom: 12px; background: #111;
  }
  #stream { display: block; width: 320px; height: 240px; object-fit: cover; }
  .info-box {
    background: #1a1a1a; border: 1px solid #333; border-radius: 6px;
    padding: 8px 20px; margin-bottom: 10px; font-size: 13px;
    color: #aaa; min-width: 220px; text-align: center;
    transition: color 0.3s, border-color 0.3s;
  }
  .grid {
    display: grid;
    grid-template-columns: repeat(3, 80px);
    grid-template-rows: repeat(3, 60px);
    gap: 6px;
  }
  button {
    background: #1e1e1e; color: #e8e8e8; border: 1px solid #444;
    border-radius: 8px; font-size: 22px; cursor: pointer;
    transition: background 0.15s;
  }
  button:hover  { background: #2a2a2a; border-color: #00e5ff; }
  button:active { background: #003344; }
  .empty { background: transparent !important; border: none !important; cursor: default !important; }
  #dist-box { margin-top: 12px; font-size: 12px; color: #666; }
</style>
</head>
<body>
<h2>&#129302; SECURITY ROBOT</h2>
<div id="stream-box">
  <img id="stream" src="" alt="Stream loading..." />
</div>
<div class="info-box" id="cmd-status">Ready</div>
<div class="info-box" id="face-status">&#128100; Face detection: active</div>
<div class="grid">
  <div class="empty"></div>
  <button onclick="cmd('forward')"  title="Forward">&#8679;</button>
  <div class="empty"></div>
  <button onclick="cmd('left')"     title="Left">&#8678;</button>
  <button onclick="cmd('stop')"     title="Stop">&#9632;</button>
  <button onclick="cmd('right')"    title="Right">&#8680;</button>
  <div class="empty"></div>
  <button onclick="cmd('backward')" title="Backward">&#8681;</button>
  <div class="empty"></div>
</div>
<div id="dist-box">Sensor: --</div>

<script>
  var host = window.location.hostname;
  document.getElementById('stream').src = 'http://' + host + ':81/stream';
  document.getElementById('stream').onerror = function() {
    this.alt = 'Stream unavailable — check port 81';
  };

  function cmd(action) {
    var st = document.getElementById('cmd-status');
    st.style.color = '#aaa';
    st.innerText = 'Sending: ' + action + '...';
    fetch('/cmd?action=' + action)
      .then(function(r) { return r.text(); })
      .then(function(t) {
        st.innerText = t;
        st.style.color = t.indexOf('BLOCKED') !== -1 ? '#ff4444' : '#00cc88';
      })
      .catch(function() {
        st.innerText = 'Error — robot may have disconnected';
        st.style.color = '#ff4444';
      });
  }

  // Poll sensor every 1.5s
  setInterval(function() {
    fetch('/dist')
      .then(function(r) { return r.text(); })
      .then(function(t) {
        document.getElementById('dist-box').innerText = 'Sensor: ' + t;
      })
      .catch(function() {});
  }, 1500);

  // Poll face detection every 5s (matches FACE_CHECK_INTERVAL_MS + processing time)
  setInterval(function() {
    var fd = document.getElementById('face-status');
    fetch('/face')
      .then(function(r) { return r.text(); })
      .then(function(t) {
        if (t === 'FACE') {
          fd.innerText = '🚨 FACE DETECTED!';
          fd.style.color = '#ff4444';
          fd.style.borderColor = '#ff4444';
          setTimeout(function() {
            fd.innerText = '👤 Face detection: active';
            fd.style.color = '#aaa';
            fd.style.borderColor = '#333';
          }, 4000);
        } else {
          fd.innerText = '👤 Face: none detected';
          fd.style.color = '#666';
        }
      })
      .catch(function() {});
  }, 5000);
</script>
</body>
</html>
)rawliteral";

//  HTTP HANDLERS

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
  httpd_resp_send(req, PAGE_HTML, strlen(PAGE_HTML));
  return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char query[64]  = {0};
  char action[32] = {0};
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
      httpd_query_key_value(query, "action", action, sizeof(action)) != ESP_OK) {
    httpd_resp_send(req, "Bad request", -1);
    return ESP_OK;
  }

  Serial.printf("[CMD] %s\n", action);
  const char* resp = "Unknown command";

  if      (strcmp(action, "stop")     == 0) { stopMotors();   resp = "Stopped"; }
  else if (strcmp(action, "backward") == 0) { moveBackward(); resp = "Backward (400ms)"; }
  else if (strcmp(action, "forward")  == 0) {
    if (pathIsClear()) { moveForward(); resp = "Forward (400ms)"; }
    else               { warningBeep(); stopMotors(); resp = "BLOCKED: Obstacle < 20cm!"; }
  }
  else if (strcmp(action, "left") == 0) {
    if (pathIsClear()) { turnLeft();  resp = "Left (400ms)"; }
    else               { warningBeep(); stopMotors(); resp = "BLOCKED: Obstacle < 20cm!"; }
  }
  else if (strcmp(action, "right") == 0) {
    if (pathIsClear()) { turnRight(); resp = "Right (400ms)"; }
    else               { warningBeep(); stopMotors(); resp = "BLOCKED: Obstacle < 20cm!"; }
  }

  httpd_resp_send(req, resp, -1);
  return ESP_OK;
}

static esp_err_t dist_handler(httpd_req_t *req) {
  long dist = getDistanceCM();
  char buf[32];
  if (dist >= 999) snprintf(buf, sizeof(buf), "Clear (>400cm)");
  else             snprintf(buf, sizeof(buf), "%ld cm %s",
                            dist, dist < SAFE_DISTANCE_CM ? "CLOSE" : "OK");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_send(req, buf, -1);
  return ESP_OK;
}

// /face — runs face detection right now, returns "FACE" or "NONE"
// Called by browser JS every 5 seconds.
// Also triggers buzzer directly when called from browser.
static esp_err_t face_handler(httpd_req_t *req) {
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  bool detected = checkForFace();
  if (detected) {
    faceBeep();
    Serial.println("[FACE] *** FACE DETECTED — BUZZER TRIGGERED ***");
    httpd_resp_send(req, "FACE", -1);
  } else {
    httpd_resp_send(req, "NONE", -1);
  }
  return ESP_OK;
}


//  MJPEG STREAM — port 81

#define STREAM_BOUNDARY "robotframe"
static const char* STREAM_CT  = "multipart/x-mixed-replace;boundary=" STREAM_BOUNDARY;
static const char* STREAM_SEP = "\r\n--" STREAM_BOUNDARY "\r\n";
static const char* STREAM_HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb      = NULL;
  uint8_t     *jpg_buf = NULL;
  size_t       jpg_len = 0;
  char         hdr_buf[64];
  esp_err_t    res     = ESP_OK;

  httpd_resp_set_type(req, STREAM_CT);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }

    // Camera is configured PIXFORMAT_JPEG so fb->buf is already a JPEG
    jpg_len = fb->len;
    jpg_buf = fb->buf;

    res = httpd_resp_send_chunk(req, STREAM_SEP, strlen(STREAM_SEP));
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    size_t hlen = snprintf(hdr_buf, sizeof(hdr_buf), STREAM_HDR, jpg_len);
    res = httpd_resp_send_chunk(req, hdr_buf, hlen);
    if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

    res = httpd_resp_send_chunk(req, (const char*)jpg_buf, jpg_len);
    esp_camera_fb_return(fb);
    fb = NULL;

    if (res != ESP_OK) break;
  }
  return res;
}

//  START SERVERS

void startServers() {
  httpd_config_t cfg80   = HTTPD_DEFAULT_CONFIG();
  cfg80.server_port      = 80;
  cfg80.ctrl_port        = 32768;
  cfg80.max_uri_handlers = 8;

  httpd_uri_t uri_index = { "/",     HTTP_GET, index_handler, NULL };
  httpd_uri_t uri_cmd   = { "/cmd",  HTTP_GET, cmd_handler,   NULL };
  httpd_uri_t uri_dist  = { "/dist", HTTP_GET, dist_handler,  NULL };
  httpd_uri_t uri_face  = { "/face", HTTP_GET, face_handler,  NULL };  // ★ new

  if (httpd_start(&control_httpd, &cfg80) == ESP_OK) {
    httpd_register_uri_handler(control_httpd, &uri_index);
    httpd_register_uri_handler(control_httpd, &uri_cmd);
    httpd_register_uri_handler(control_httpd, &uri_dist);
    httpd_register_uri_handler(control_httpd, &uri_face);
    Serial.println("[HTTP] Control server on port 80");
  } else {
    Serial.println("[HTTP] Control server FAILED");
  }

  httpd_config_t cfg81   = HTTPD_DEFAULT_CONFIG();
  cfg81.server_port      = 81;
  cfg81.ctrl_port        = 32769;
  cfg81.max_uri_handlers = 2;

  httpd_uri_t uri_stream = { "/stream", HTTP_GET, stream_handler, NULL };
  if (httpd_start(&stream_httpd, &cfg81) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &uri_stream);
    Serial.println("[HTTP] Stream server on port 81");
  } else {
    Serial.println("[HTTP] Stream server FAILED");
  }
}

//  CAMERA INIT

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;

  // PIXFORMAT_JPEG: stream works directly, face detection converts via fmt2rgb888()
  // Do NOT use PIXFORMAT_GRAYSCALE here — fmt2rgb888 needs JPEG or RGB565 as input
  config.pixel_format = PIXFORMAT_JPEG;

  // QVGA (320×240) — best balance: stream is fast, detection accuracy is acceptable
  // Do NOT use UXGA/SXGA for detection — conversion will run out of PSRAM
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;   // 10 = good quality; lower number = better quality
  config.fb_count     = 1;    // 1 frame buffer — face detection needs exclusive access

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[CAM] Init failed: 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (s) {
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
    s->set_gainceiling(s, (gainceiling_t)2);
  }

  Serial.println("[CAM] Initialized OK");
  return true;
}


//  SETUP

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Security Robot Boot (Core 1.0.6) ===");

  // Motor pins — set LOW first (GPIO12 is a strapping pin — must be LOW at boot)
  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  startupBeep();

  //  Configure face detection 
  // These are Espressif's recommended values from the 1.0.6 example.
  // Adjust ONLY if you get too many false positives (raise .score values)
  // or miss real faces (lower .score, reduce min_face).
  memset(&mtmn_config, 0, sizeof(mtmn_config));
  mtmn_config.type          = FAST;   // FAST = one-shot; NORMAL = slower, more accurate
  mtmn_config.min_face      = 80;     // Minimum face size in pixels (80 = ~1m range at QVGA)
  mtmn_config.pyramid       = 0.707f; // Scale factor between detection passes
  mtmn_config.pyramid_times = 4;      // Number of scale passes (more = catches more sizes)

  // P-net: coarse candidate filtering
  mtmn_config.p_threshold.score            = 0.6f;
  mtmn_config.p_threshold.nms              = 0.7f;
  mtmn_config.p_threshold.candidate_number = 20;

  // R-net: refines P-net candidates
  mtmn_config.r_threshold.score            = 0.7f;
  mtmn_config.r_threshold.nms              = 0.7f;
  mtmn_config.r_threshold.candidate_number = 10;

  // O-net: final verification — highest confidence stage
  mtmn_config.o_threshold.score            = 0.7f;
  mtmn_config.o_threshold.nms              = 0.4f;
  mtmn_config.o_threshold.candidate_number = 1;

  if (!initCamera()) {
    Serial.println("[BOOT] Camera failed — halting");
    errorBeep();
    while (true) delay(1000);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.printf("[WiFi] Connecting to %s", ssid);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500); Serial.print("."); attempts++;
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[WiFi] FAILED — check credentials");
    errorBeep();
    while (true) delay(1000);
  }

  Serial.printf("[WiFi] Connected! IP: %s\n", WiFi.localIP().toString().c_str());
  connectedBeep();

  startServers();

  Serial.printf("[BOOT] Control: http://%s/\n",          WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Stream:  http://%s:81/stream\n", WiFi.localIP().toString().c_str());
  Serial.printf("[BOOT] Face:    http://%s/face\n",      WiFi.localIP().toString().c_str());
  Serial.println("[BOOT] Ready!");
}


//  LOOP


void loop() {
  //  Passive obstacle safety ─
  static unsigned long lastObstacleCheck = 0;
  if (millis() - lastObstacleCheck > 300) {
    lastObstacleCheck = millis();
    if (getDistanceCM() < SAFE_DISTANCE_CM) stopMotors();
  }

  //  Periodic background face detection ─
  // Runs checkForFace() every FACE_CHECK_INTERVAL_MS (default 4 seconds).
  // This path operates even when no browser is open — pure autonomous security.
  // Note: checkForFace() takes ~600-900ms. The stream will stutter during this.
  // That is expected — both the stream and detection use the same camera hardware.
  if (millis() - lastFaceCheck > FACE_CHECK_INTERVAL_MS) {
    lastFaceCheck = millis();
    Serial.println("[FACE] Background check...");
    if (checkForFace()) {
      faceBeep();
      Serial.println("[FACE] *** FACE DETECTED (background) ***");
    }
  }
}