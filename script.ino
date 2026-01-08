#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include <ctype.h>
#include "driver/ledc.h"

// =====================
// WIFI CONFIG
// =====================
const char* WIFI_SSID = "ESP32-CAM Robot";
const char* WIFI_PASS = "";

bool cameraReady = false;

// =====================
// MOTOR PINS (STOCK FIRMWARE)
// =====================
#define MOTOR_LEFT_M0 13
#define MOTOR_LEFT_M1 12
#define MOTOR_RIGHT_M0 14
#define MOTOR_RIGHT_M1 15

#define LED_PIN 4

const int kMotorPwmFrequency = 2000;
const int kMotorPwmResolution = 8;
const int kMotorLeftM0Channel = LEDC_CHANNEL_4;
const int kMotorLeftM1Channel = LEDC_CHANNEL_5;
const int kMotorRightM0Channel = LEDC_CHANNEL_6;
const int kMotorRightM1Channel = LEDC_CHANNEL_7;
int motorSpeed = 150;

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

void setMotorDuty(ledc_channel_t channel, uint32_t duty) {
  ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void resetMotorOutputs() {
  setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM0Channel), 0);
  setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM1Channel), 0);
  setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM0Channel), 0);
  setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM1Channel), 0);
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

uint32_t percentToDuty(float percent) {
  percent = max(0.0f, min(percent, 100.0f));
  return static_cast<uint32_t>(map(static_cast<int>(percent), 0, 100, 0, 255));
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

  uint32_t leftDuty = 0;
  uint32_t rightDuty = 0;
  if (leftDir != 0) {
    leftDuty = percentToDuty(min(abs(leftCommand), 1.0f) * 100.0f) * motorSpeed / 255;
  }
  if (rightDir != 0) {
    rightDuty = percentToDuty(min(abs(rightCommand), 1.0f) * 100.0f) * motorSpeed / 255;
  }

  if (leftDir > 0) {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM0Channel), 0);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM1Channel), leftDuty);
  } else if (leftDir < 0) {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM0Channel), leftDuty);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM1Channel), 0);
  } else {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM0Channel), 0);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorLeftM1Channel), 0);
  }

  if (rightDir > 0) {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM0Channel), 0);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM1Channel), rightDuty);
  } else if (rightDir < 0) {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM0Channel), rightDuty);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM1Channel), 0);
  } else {
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM0Channel), 0);
    setMotorDuty(static_cast<ledc_channel_t>(kMotorRightM1Channel), 0);
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
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    logEvent("camera.init_failed");
    return false;
  }

  sensor_t* s = esp_camera_sensor_get();
  if (s) {
    currentFrameSize = FRAMESIZE_QVGA;
    jpegQuality = config.jpeg_quality;
    s->set_framesize(s, currentFrameSize);
    s->set_quality(s, jpegQuality);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 1);
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
  Serial.setDebugOutput(true);

  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_1,
    .freq_hz = kMotorPwmFrequency,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&ledc_timer);

  ledc_channel_config_t ledc_channel[4] = {
    {
      .gpio_num = MOTOR_LEFT_M0,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = static_cast<ledc_channel_t>(kMotorLeftM0Channel),
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,
      .duty = 0,
      .hpoint = 0
    },
    {
      .gpio_num = MOTOR_LEFT_M1,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = static_cast<ledc_channel_t>(kMotorLeftM1Channel),
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,
      .duty = 0,
      .hpoint = 0
    },
    {
      .gpio_num = MOTOR_RIGHT_M0,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = static_cast<ledc_channel_t>(kMotorRightM0Channel),
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,
      .duty = 0,
      .hpoint = 0
    },
    {
      .gpio_num = MOTOR_RIGHT_M1,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = static_cast<ledc_channel_t>(kMotorRightM1Channel),
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER_1,
      .duty = 0,
      .hpoint = 0
    }
  };

  for (int i = 0; i < 4; i++) {
    ledc_channel_config(&ledc_channel[i]);
  }

  pinMode(33, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  stopMotors();

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  IPAddress apIp = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(apIp);
  logEvent("wifi.ap_ready");

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
