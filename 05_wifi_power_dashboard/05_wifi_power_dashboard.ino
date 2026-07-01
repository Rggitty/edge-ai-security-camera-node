#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <WebServer.h>

// =======================
// Wi-Fi credentials
// =======================
const char* ssid = "YOUR_WIFI_NAME";
const char* password = "YOUR_WIFI_PASSWORD";

// =======================
// Web server
// =======================
WebServer server(80);

// =======================
// PIR motion sensor
// =======================
#define PIR_PIN 4

// =======================
// INA219 I2C pins
// =======================
#define I2C_SDA 13
#define I2C_SCL 14

Adafruit_INA219 ina219;

// =======================
// Camera pin mapping
// =======================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    33
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

// =======================
// State variables
// =======================
unsigned long eventCount = 0;
unsigned long lastCaptureTime = 0;
const unsigned long cooldownMs = 5000;
int lastPirState = LOW;

float busVoltage = 0;
float shuntVoltage = 0;
float loadVoltage = 0;
float current_mA = 0;
float power_mW = 0;

unsigned long lastPowerRead = 0;
unsigned long lastCaptureMicros = 0;
size_t lastImageBytes = 0;
String lastState = "booting";

void readPower() {
  busVoltage = ina219.getBusVoltage_V();
  shuntVoltage = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadVoltage = busVoltage + (shuntVoltage / 1000.0);
  lastPowerRead = millis();
}

bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
  } else {
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  return true;
}

void captureImageEvent() {
  eventCount++;
  lastState = "motion_capture";

  unsigned long startMicros = micros();
  camera_fb_t *fb = esp_camera_fb_get();
  lastCaptureMicros = micros() - startMicros;

  if (!fb) {
    lastState = "capture_failed";
    lastImageBytes = 0;
    return;
  }

  lastImageBytes = fb->len;
  esp_camera_fb_return(fb);

  readPower();
  lastState = "capture_ok";
}

String htmlPage() {
  String page = "";
  page += "<!DOCTYPE html><html><head>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>ESP32 Edge Camera Power Dashboard</title>";
  page += "<style>";
  page += "body{font-family:Arial;margin:25px;background:#111;color:#eee;}";
  page += ".card{background:#222;padding:18px;margin:12px 0;border-radius:12px;}";
  page += ".big{font-size:28px;font-weight:bold;}";
  page += "table{width:100%;border-collapse:collapse;}";
  page += "td{padding:8px;border-bottom:1px solid #444;}";
  page += "</style>";
  page += "<script>";
  page += "async function update(){";
  page += "let r=await fetch('/data');";
  page += "let d=await r.json();";
  page += "document.getElementById('state').innerText=d.state;";
  page += "document.getElementById('events').innerText=d.events;";
  page += "document.getElementById('bus').innerText=d.bus_V.toFixed(3)+' V';";
  page += "document.getElementById('load').innerText=d.load_V.toFixed(3)+' V';";
  page += "document.getElementById('current').innerText=d.current_mA.toFixed(3)+' mA';";
  page += "document.getElementById('power').innerText=d.power_mW.toFixed(3)+' mW';";
  page += "document.getElementById('bytes').innerText=d.image_bytes;";
  page += "document.getElementById('capture').innerText=d.capture_us+' us';";
  page += "document.getElementById('uptime').innerText=d.time_ms+' ms';";
  page += "}";
  page += "setInterval(update,1000); window.onload=update;";
  page += "</script>";
  page += "</head><body>";
  page += "<h1>Edge AI Security Camera Node</h1>";
  page += "<div class='card'><div>State</div><div class='big' id='state'>...</div></div>";
  page += "<div class='card'><table>";
  page += "<tr><td>Motion Events</td><td id='events'>...</td></tr>";
  page += "<tr><td>Bus Voltage</td><td id='bus'>...</td></tr>";
  page += "<tr><td>Load Voltage</td><td id='load'>...</td></tr>";
  page += "<tr><td>Current</td><td id='current'>...</td></tr>";
  page += "<tr><td>Power</td><td id='power'>...</td></tr>";
  page += "<tr><td>Last Image Bytes</td><td id='bytes'>...</td></tr>";
  page += "<tr><td>Last Capture Time</td><td id='capture'>...</td></tr>";
  page += "<tr><td>Uptime</td><td id='uptime'>...</td></tr>";
  page += "</table></div>";
  page += "<p>Move in front of the PIR sensor to trigger a camera capture.</p>";
  page += "</body></html>";
  return page;
}

void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleData() {
  readPower();

  String json = "{";
  json += "\"time_ms\":" + String(millis()) + ",";
  json += "\"state\":\"" + lastState + "\",";
  json += "\"events\":" + String(eventCount) + ",";
  json += "\"bus_V\":" + String(busVoltage, 3) + ",";
  json += "\"shunt_mV\":" + String(shuntVoltage, 3) + ",";
  json += "\"load_V\":" + String(loadVoltage, 3) + ",";
  json += "\"current_mA\":" + String(current_mA, 3) + ",";
  json += "\"power_mW\":" + String(power_mW, 3) + ",";
  json += "\"image_bytes\":" + String(lastImageBytes) + ",";
  json += "\"capture_us\":" + String(lastCaptureMicros);
  json += "}";

  server.send(200, "application/json", json);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE CAMERA WIFI POWER DASHBOARD ====");

  pinMode(PIR_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina219.begin(&Wire)) {
    Serial.println("ERROR: INA219 not found.");
    while (true) {
      delay(1000);
    }
  }

  ina219.setCalibration_32V_2A();
  Serial.println("INA219 initialized.");

  if (!initCamera()) {
    Serial.println("ERROR: Camera failed.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Camera initialized.");

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected.");
  Serial.print("Open this IP in your browser: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  lastState = "ready";
  readPower();
}

void loop() {
  server.handleClient();

  int pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH && lastPirState == LOW) {
    if (millis() - lastCaptureTime > cooldownMs) {
      lastCaptureTime = millis();
      captureImageEvent();
    } else {
      lastState = "cooldown";
    }
  }

  if (pirState == LOW && lastPirState == HIGH) {
    lastState = "motion_ended";
  }

  lastPirState = pirState;

  static unsigned long lastIdlePower = 0;
  if (millis() - lastIdlePower > 1000) {
    lastIdlePower = millis();
    if (lastState != "capture_ok" && lastState != "motion_capture") {
      lastState = "idle_waiting";
    }
    readPower();
  }

  delay(20);
}