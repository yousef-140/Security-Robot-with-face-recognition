#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "fd_forward.h"

const char* ssid     = "****";
const char* password = "****";

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

#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 15

#define BUZZER_PIN 4

#define TRIG_PIN 2
#define ECHO_PIN 16

#define SAFE_DISTANCE_CM  20
#define MOVE_PULSE_MS     400
#define FACE_CHECK_INTERVAL_MS  4000

static mtmn_config_t mtmn_config;
static unsigned long lastFaceCheck = 0;

httpd_handle_t control_httpd = NULL;
httpd_handle_t stream_httpd  = NULL;

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

void beep(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(offMs);
  }
}

void startupBeep()  { beep(2, 100, 100); }
void connectedBeep(){ beep(1, 600,   0); }
void warningBeep()  { beep(3,  80,  60); }
void errorBeep()    { beep(6, 200, 100); }
void faceBeep()     { beep(3, 200, 100); }

bool checkForFace() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[FACE] Frame capture failed");
    return false;
  }

  dl_matrix3du_t *rgb_buf = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!rgb_buf) {
    Serial.println("[FACE] RGB buffer alloc failed");
    esp_camera_fb_return(fb);
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, fb->format, rgb_buf->item);
  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("[FACE] Conversion failed");
    dl_matrix3du_free(rgb_buf);
    return false;
  }

  box_array_t *net_boxes = face_detect(rgb_buf, &mtmn_config);
  dl_matrix3du_free(rgb_buf);

  if (net_boxes) {
    Serial.printf("[FACE] Face(s) detected: %d\n", net_boxes->len);
    free(net_boxes->box);
    if (net_boxes->landmark) free(net_boxes->landmark);
    free(net_boxes);
    return true;
  }

  Serial.println("[FACE] No face");
  return false;
}

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
</style>
</head>
<body>
<h2>&#129302; SECURITY ROBOT</h2>
<div id="stream-box">
  <img id="stream" src="" alt="Stream loading..." />
</div>
<div class="info-box" id="cmd-status">Ready</div>
<div class="info-box" id="face-status">&#128100; Face detection: active</div>
<div class="info-box" id="dist-box">&#128225; Sensor: --</div>
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

<script>
  var host = window.location.hostname;
  document.getElementById('stream').src = 'http://' + host + ':81/stream';
  document.getElementById('stream').onerror = function() {
    this.alt = 'Stream unavailable';
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

  setInterval(function() {
    fetch('/dist')
      .then(function(r) { return r.text(); })
      .then(function(t) {
        var box = document.getElementById('dist-box');
        box.innerText = '📡 Sensor: ' + t;
        if (t.indexOf('CLOSE') !== -1) {
          box.style.color       = '#ff4444';
          box.style.borderColor = '#ff4444';
        } else {
          box.style.color       = '#00cc88';
          box.style.borderColor = '#333';
        }
      })
      .catch(function() {});
  }, 1500);

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

void startServers() {
  httpd_config_t cfg80   = HTTPD_DEFAULT_CONFIG();
  cfg80.server_port      = 80;
  cfg80.ctrl_port        = 32768;
  cfg80.max_uri_handlers = 8;

  httpd_uri_t uri_index = { "/",     HTTP_GET, index_handler, NULL };
  httpd_uri_t uri_cmd   = { "/cmd",  HTTP_GET, cmd_handler,   NULL };
  httpd_uri_t uri_dist  = { "/dist", HTTP_GET, dist_handler,  NULL };
  httpd_uri_t uri_face  = { "/face", HTTP_GET, face_handler,  NULL };

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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count     = 1;

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

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== Security Robot Boot ===");

  pinMode(IN1, OUTPUT); digitalWrite(IN1, LOW);
  pinMode(IN2, OUTPUT); digitalWrite(IN2, LOW);
  pinMode(IN3, OUTPUT); digitalWrite(IN3, LOW);
  pinMode(IN4, OUTPUT); digitalWrite(IN4, LOW);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);

  startupBeep();

  memset(&mtmn_config, 0, sizeof(mtmn_config));
  mtmn_config.type          = FAST;
  mtmn_config.min_face      = 80;
  mtmn_config.pyramid       = 0.707f;
  mtmn_config.pyramid_times = 4;

  mtmn_config.p_threshold.score            = 0.6f;
  mtmn_config.p_threshold.nms              = 0.7f;
  mtmn_config.p_threshold.candidate_number = 20;

  mtmn_config.r_threshold.score            = 0.7f;
  mtmn_config.r_threshold.nms              = 0.7f;
  mtmn_config.r_threshold.candidate_number = 10;

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

void loop() {
  static unsigned long lastObstacleCheck = 0;
  if (millis() - lastObstacleCheck > 300) {
    lastObstacleCheck = millis();
    long d = getDistanceCM();
    if (d < SAFE_DISTANCE_CM) {
      stopMotors();
      warningBeep();
    }
  }

  if (millis() - lastFaceCheck > FACE_CHECK_INTERVAL_MS) {
    lastFaceCheck = millis();
    Serial.println("[FACE] Background check...");
    if (checkForFace()) {
      faceBeep();
      Serial.println("[FACE] *** FACE DETECTED (background) ***");
    }
  }
}
