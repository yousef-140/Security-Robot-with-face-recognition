/*

   Security Robot — ESP32-CAM AI Thinker (CAMERA TEMP DISABLED)

 */

///  CAMERA DISABLED 
// #include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
// #include "fd_forward.h"


 
const char* ssid     = "***";
const char* password = "***";


/// ===== CAMERA PINS DISABLED =====
/*
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
*/
/

// ── Motor pins 
#define IN1 12
#define IN2 13
#define IN3 14
#define IN4 15

// ── Buzzer
#define BUZZER_PIN 4

// ── HC-SR04
#define TRIG_PIN 2
#define ECHO_PIN 16

#define SAFE_DISTANCE_CM  20
#define MOVE_PULSE_MS     400

///  FACE DETECTION DISABLED 
// static mtmn_config_t mtmn_config;
// static unsigned long lastFaceCheck = 0;

httpd_handle_t control_httpd = NULL;
// httpd_handle_t stream_httpd  = NULL;
// MOTOR

void stopMotors() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(MOVE_PULSE_MS); stopMotors();
}

void moveBackward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(MOVE_PULSE_MS); stopMotors();
}

void turnLeft() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  delay(MOVE_PULSE_MS); stopMotors();
}

void turnRight() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  delay(MOVE_PULSE_MS); stopMotors();
}
// ULTRASONIC

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
// BUZZER

void beep(int count, int onMs, int offMs) {
  for (int i = 0; i < count; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(onMs);
    digitalWrite(BUZZER_PIN, LOW);
    if (i < count - 1) delay(offMs);
  }
}

void startupBeep()  { beep(2, 100, 100); }
void connectedBeep(){ beep(1, 600, 0);   }
void warningBeep()  { beep(3, 80,  60);  }

/// ===== FACE BUZZER DISABLED =====
// void faceBeep() { beep(3,200,100); }


/// ===== FACE FUNCTION DISABLED =====
/*
bool checkForFace() {
  // disabled
  return false;
}
*/
/// HTML

static const char PROGMEM PAGE_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html>
<body style="background:#111;color:white;text-align:center;">
<h2>Robot Control (NO CAMERA)</h2>
<button onclick="cmd('forward')">↑</button><br>
<button onclick="cmd('left')">←</button>
<button onclick="cmd('stop')">■</button>
<button onclick="cmd('right')">→</button><br>
<button onclick="cmd('backward')">↓</button>

<script>
function cmd(a){
 fetch('/cmd?action='+a)
}
</script>
</body>
</html>
)rawliteral";
// HTTP

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_send(req, PAGE_HTML, strlen(PAGE_HTML));
  return ESP_OK;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char query[64]={0}, action[32]={0};

  if (httpd_req_get_url_query_str(req, query, sizeof(query))==ESP_OK)
    httpd_query_key_value(query,"action",action,sizeof(action));

  if (!strcmp(action,"forward")) {
    if(pathIsClear()) moveForward(); else warningBeep();
  }
  else if (!strcmp(action,"backward")) moveBackward();
  else if (!strcmp(action,"left")) {
    if(pathIsClear()) turnLeft(); else warningBeep();
  }
  else if (!strcmp(action,"right")) {
    if(pathIsClear()) turnRight(); else warningBeep();
  }
  else stopMotors();

  httpd_resp_send(req,"OK",-1);
  return ESP_OK;
}
// SERVER

void startServers() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();

  httpd_uri_t uri_index = { "/", HTTP_GET, index_handler };
  httpd_uri_t uri_cmd   = { "/cmd", HTTP_GET, cmd_handler };

  if (httpd_start(&control_httpd, &cfg)==ESP_OK) {
    httpd_register_uri_handler(control_httpd,&uri_index);
    httpd_register_uri_handler(control_httpd,&uri_cmd);
  }
}
// SETUP

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  startupBeep();

  /// ===== CAMERA INIT DISABLED =====
  /*
  if (!initCamera()) {
    errorBeep();
    while(true);
  }
  */
  
  WiFi.begin(ssid,password);

  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println(WiFi.localIP());

  startServers();
}
// LOOP

void loop() {

  /// ===== FACE LOOP DISABLED =====
  /*
  if (millis()-lastFaceCheck>4000){
    if(checkForFace()) faceBeep();
  }
  */
  
  if (getDistanceCM()<SAFE_DISTANCE_CM)
    stopMotors();
}