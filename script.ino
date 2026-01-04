#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

// =====================
// WIFI CONFIG
// =====================
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

// =====================
// MOTOR PINS (PLACEHOLDERS)
// =====================
#define MOTOR_L_FWD 12
#define MOTOR_L_REV 13
#define MOTOR_R_FWD 14
#define MOTOR_R_REV 15

// =====================
// SERVER
// =====================
WebServer server(80);

// =====================
// MOTION STATE
// =====================
struct MotionState {
  bool active = false;
  float linear = 0.0;
  float angular = 0.0;
  unsigned long since_ms = 0;
};

MotionState motion;
String eventLog[20];
int eventIndex = 0;

// =====================
// CAMERA CONFIG STATE
// =====================
framesize_t currentFrameSize = FRAMESIZE_QVGA;
int jpegQuality = 12;

// =====================
// UTILS
// =====================
void logEvent(String msg) {
  eventLog[eventIndex % 20] = msg;
  eventIndex++;
}

void stopMotors() {
  digitalWrite(MOTOR_L_FWD, LOW);
  digitalWrite(MOTOR_L_REV, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);
  digitalWrite(MOTOR_R_REV, LOW);
  motion.active = false;
}

// =====================
// CAMERA INIT
// =====================
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sccb_sda = 26;
  config.pin_sccb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = currentFrameSize;
  config.jpeg_quality = jpegQuality;
  config.fb_count = 1;

  esp_camera_init(&config);
}

// =====================
// CAMERA ENDPOINTS
// =====================
void handleCameraFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "camera capture failed");
    return;
  }

  server.sendHeader("Content-Type", "image/jpeg");
  server.sendHeader("Content-Length", String(fb->len));
  server.send(200);
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void handleCameraConfig() {
  String json = "{";
  json += "\"frame_size\":\"QVGA\",";
  json += "\"jpeg_quality\":" + String(jpegQuality) + ",";
  json += "\"pixel_format\":\"JPEG\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleCameraStream() {
  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) continue;

    client.println("--frame");
    client.println("Content-Type: image/jpeg");
    client.println("Content-Length: " + String(fb->len));
    client.println();
    client.write(fb->buf, fb->len);
    client.println();

    esp_camera_fb_return(fb);
    delay(80);
  }
}

// =====================
// SYSTEM ENDPOINTS
// =====================
void handleCapabilities() {
  server.send(200, "application/json",
    R"({
      "motion": true,
      "camera": true,
      "battery": false,
      "proximity": false,
      "pose": false,
      "charging": false
    })");
}

void handleHealth() {
  String json = "{";
  json += "\"uptime_ms\":" + String(millis()) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap());
  json += "}";
  server.send(200, "application/json", json);
}

void handleEvents() {
  String json = "[";
  for (int i = 0; i < 20; i++) {
    if (eventLog[i].length() > 0) {
      json += "\"" + eventLog[i] + "\",";
    }
  }
  if (json.endsWith(",")) json.remove(json.length() - 1);
  json += "]";
  server.send(200, "application/json", json);
}

// =====================
// MOTION ENDPOINTS
// =====================
void handleMotionState() {
  String json = "{";
  json += "\"active\":" + String(motion.active ? "true" : "false") + ",";
  json += "\"linear\":" + String(motion.linear) + ",";
  json += "\"angular\":" + String(motion.angular) + ",";
  json += "\"since_ms\":" + String(motion.since_ms);
  json += "}";
  server.send(200, "application/json", json);
}

void handleMotionStop() {
  stopMotors();
  logEvent("motion.stop");
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleMotionDrive() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"missing body\"}");
    return;
  }

  String body = server.arg("plain");

  float linear = body.indexOf("linear") >= 0
    ? body.substring(body.indexOf("linear") + 7).toFloat()
    : 0;

  float angular = body.indexOf("angular") >= 0
    ? body.substring(body.indexOf("angular") + 8).toFloat()
    : 0;

  motion.linear = linear;
  motion.angular = angular;
  motion.active = true;
  motion.since_ms = millis();

  stopMotors();

  if (linear > 0) {
    digitalWrite(MOTOR_L_FWD, HIGH);
    digitalWrite(MOTOR_R_FWD, HIGH);
  } else if (linear < 0) {
    digitalWrite(MOTOR_L_REV, HIGH);
    digitalWrite(MOTOR_R_REV, HIGH);
  }

  if (angular > 0) {
    digitalWrite(MOTOR_L_FWD, HIGH);
    digitalWrite(MOTOR_R_REV, HIGH);
  } else if (angular < 0) {
    digitalWrite(MOTOR_L_REV, HIGH);
    digitalWrite(MOTOR_R_FWD, HIGH);
  }

  logEvent("motion.drive");
  server.send(200, "application/json", "{\"ok\":true}");
}

// =====================
// SETUP
// =====================
void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_L_FWD, OUTPUT);
  pinMode(MOTOR_L_REV, OUTPUT);
  pinMode(MOTOR_R_FWD, OUTPUT);
  pinMode(MOTOR_R_REV, OUTPUT);

  stopMotors();
  initCamera();

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  logEvent("boot");

  server.on("/capabilities", handleCapabilities);
  server.on("/health", handleHealth);
  server.on("/events", handleEvents);

  server.on("/camera/frame", HTTP_GET, handleCameraFrame);
  server.on("/camera/config", HTTP_GET, handleCameraConfig);
  server.on("/camera/stream", HTTP_GET, handleCameraStream);

  server.on("/motion/state", HTTP_GET, handleMotionState);
  server.on("/motion/stop", HTTP_POST, handleMotionStop);
  server.on("/motion/drive", HTTP_POST, handleMotionDrive);

  server.begin();
}

// =====================
// LOOP
// =====================
void loop() {
  server.handleClient();
}
