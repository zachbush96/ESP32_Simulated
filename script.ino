#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <ctype.h>

// =====================
// WIFI CONFIG
// =====================
const char* WIFI_SSID = "YOUR_WIFI";
const char* WIFI_PASS = "YOUR_PASSWORD";

bool cameraReady = false;

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
  String status = "idle";
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
  eventLog[eventIndex % 20] = String(millis()) + ":" + msg;
  eventIndex++;
}

String frameSizeToString(framesize_t frameSize) {
  switch (frameSize) {
    case FRAMESIZE_QQVGA: return "QQVGA";
    case FRAMESIZE_QVGA: return "QVGA";
    case FRAMESIZE_VGA: return "VGA";
    case FRAMESIZE_SVGA: return "SVGA";
    case FRAMESIZE_XGA: return "XGA";
    case FRAMESIZE_SXGA: return "SXGA";
    case FRAMESIZE_UXGA: return "UXGA";
    default: return "UNKNOWN";
  }
}

void resetMotorOutputs() {
  digitalWrite(MOTOR_L_FWD, LOW);
  digitalWrite(MOTOR_L_REV, LOW);
  digitalWrite(MOTOR_R_FWD, LOW);
  digitalWrite(MOTOR_R_REV, LOW);
}

void stopMotors() {
  resetMotorOutputs();
  motion.active = false;
  motion.linear = 0.0;
  motion.angular = 0.0;
  motion.status = "stopped";
  motion.since_ms = millis();
}

bool isNumericChar(char c) {
  return (c >= '0' && c <= '9') || c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E';
}

bool extractJsonFloat(const String& body, const char* key, float& valueOut) {
  String pattern = "\"" + String(key) + "\"";
  int keyPos = body.indexOf(pattern);
  if (keyPos < 0) {
    return false;
  }

  int colonPos = body.indexOf(":", keyPos + pattern.length());
  if (colonPos < 0) {
    return false;
  }

  int start = colonPos + 1;
  while (start < body.length() && isspace(static_cast<unsigned char>(body[start]))) {
    start++;
  }

  int end = start;
  while (end < body.length() && isNumericChar(body[end])) {
    end++;
  }

  if (start == end) {
    return false;
  }

  valueOut = body.substring(start, end).toFloat();
  return true;
}

void driveDifferential(float linear, float angular) {
  const float DEADZONE = 0.05f;
  float leftCommand = linear - angular;
  float rightCommand = linear + angular;

  int leftDir = 0;
  int rightDir = 0;

  if (leftCommand > DEADZONE) {
    leftDir = 1;
  } else if (leftCommand < -DEADZONE) {
    leftDir = -1;
  }

  if (rightCommand > DEADZONE) {
    rightDir = 1;
  } else if (rightCommand < -DEADZONE) {
    rightDir = -1;
  }

  resetMotorOutputs();

  if (leftDir > 0) {
    digitalWrite(MOTOR_L_FWD, HIGH);
  } else if (leftDir < 0) {
    digitalWrite(MOTOR_L_REV, HIGH);
  }

  if (rightDir > 0) {
    digitalWrite(MOTOR_R_FWD, HIGH);
  } else if (rightDir < 0) {
    digitalWrite(MOTOR_R_REV, HIGH);
  }

  motion.active = (leftDir != 0) || (rightDir != 0);
  motion.status = motion.active ? "driving" : "stopped";
  motion.linear = linear;
  motion.angular = angular;
  motion.since_ms = millis();
}

// =====================
// CAMERA INIT
// =====================
bool initCamera() {
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

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    logEvent("camera.init_failed");
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    s->set_framesize(s, currentFrameSize);
    s->set_quality(s, jpegQuality);
  }

  logEvent("camera.ready");
  return true;
}

// =====================
// CAMERA ENDPOINTS
// =====================
void handleCameraFrame() {
  if (!cameraReady) {
    server.send(503, "application/json", "{\"error\":\"camera not ready\"}");
    return;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    server.send(500, "text/plain", "camera capture failed");
    return;
  }

  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);
}

void handleCameraConfig() {
  String json = "{";
  json += "\"frame_size\":\"" + frameSizeToString(currentFrameSize) + "\",";
  json += "\"jpeg_quality\":" + String(jpegQuality) + ",";
  json += "\"pixel_format\":\"JPEG\",";
  json += "\"ready\":" + String(cameraReady ? "true" : "false");
  json += "}";
  server.send(200, "application/json", json);
}

void handleCameraStream() {
  if (!cameraReady) {
    server.send(503, "application/json", "{\"error\":\"camera not ready\"}");
    return;
  }

  WiFiClient client = server.client();
  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
  client.println();

  while (client.connected()) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

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
  String json = "{";
  json += "\"motion\":true,";
  json += "\"camera\":" + String(cameraReady ? "true" : "false") + ",";
  json += "\"battery\":false,";
  json += "\"proximity\":false,";
  json += "\"pose\":false,";
  json += "\"charging\":false";
  json += "}";
  server.send(200, "application/json", json);
}

void handleHealth() {
  String json = "{";
  json += "\"uptime_ms\":" + String(millis()) + ",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"wifi_rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleEvents() {
  String json = "{\"events\":[";
  for (int i = 0; i < 20; i++) {
    if (eventLog[i].length() > 0) {
      json += "\"" + eventLog[i] + "\",";
    }
  }
  if (json.endsWith(",")) json.remove(json.length() - 1);
  json += "],";
  json += "\"count\":" + String(eventIndex < 20 ? eventIndex : 20);
  json += "}";
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
  json += "\"since_ms\":" + String(motion.since_ms) + ",";
  json += "\"status\":\"" + motion.status + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleMotionStop() {
  stopMotors();
  logEvent("motion.stop");
  server.send(200, "application/json", "{\"ok\":true,\"status\":\"stopped\"}");
}

void handleMotionDrive() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"error\":\"missing body\"}");
    return;
  }

  String body = server.arg("plain");

  float linear = 0.0;
  float angular = 0.0;

  bool hasLinear = extractJsonFloat(body, "linear", linear);
  bool hasAngular = extractJsonFloat(body, "angular", angular);

  if (!hasLinear && !hasAngular) {
    server.send(400, "application/json", "{\"error\":\"missing linear/angular\"}");
    return;
  }

  driveDifferential(linear, angular);

  logEvent("motion.drive l=" + String(linear) + " a=" + String(angular));
  String json = "{";
  json += "\"ok\":true,";
  json += "\"status\":\"" + motion.status + "\",";
  json += "\"since_ms\":" + String(motion.since_ms);
  json += "}";
  server.send(200, "application/json", json);
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

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  WiFi.setSleep(false);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  logEvent("wifi.connected");

  cameraReady = initCamera();

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
