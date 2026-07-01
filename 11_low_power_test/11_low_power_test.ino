#include <esp32_person_detector_v2_inferencing.h>

#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_heap_caps.h"

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// =======================
// Wi-Fi credentials
// Wi-Fi is OFF during measurement.
// It only turns ON after the test to show the report.
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
// SunFounder ESP32 Camera Extension OV2640 pin mapping
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
// Edge Impulse AI settings
// =======================
#define IMG_W EI_CLASSIFIER_INPUT_WIDTH
#define IMG_H EI_CLASSIFIER_INPUT_HEIGHT
#define IMG_BYTES (IMG_W * IMG_H * 3)

const float PERSON_THRESHOLD = 0.80;
const float NO_PERSON_THRESHOLD = 0.80;

const unsigned long idleMeasureMs = 30000;
const unsigned long eventMeasureMs = 10000;

static bool debug_nn = false;
static uint8_t *aiInput = nullptr;

// =======================
// Live readings
// =======================
float busVoltage = 0;
float shuntVoltage = 0;
float loadVoltage = 0;
float current_mA = 0;
float power_mW = 0;

// =======================
// Power stats struct
// =======================
struct PowerStats {
  float peakCurrent_mA;
  float peakPower_mW;
  float minLoadVoltage;
  double energy_mWh;
  double currentTimeSum;
  double powerTimeSum;
  double metricTimeMs;
  unsigned long lastSampleMs;
};

PowerStats idleStats;
PowerStats eventStats;

// =======================
// AI/event results
// =======================
String aiDecision = "not_run";
String aiBestLabel = "none";
float personScore = 0.0;
float noPersonScore = 0.0;
float aiConfidence = 0.0;
unsigned long aiInferenceMs = 0;
unsigned long captureUs = 0;
size_t frameBytes = 0;

bool testComplete = false;
bool wifiStarted = false;

unsigned long idleStartedAt = 0;
unsigned long idleEndedAt = 0;
unsigned long eventStartedAt = 0;
unsigned long eventEndedAt = 0;

// =======================
// Helpers
// =======================
void resetPowerStats(PowerStats &s) {
  s.peakCurrent_mA = 0;
  s.peakPower_mW = 0;
  s.minLoadVoltage = 999;
  s.energy_mWh = 0;
  s.currentTimeSum = 0;
  s.powerTimeSum = 0;
  s.metricTimeMs = 0;
  s.lastSampleMs = 0;
}

float avgCurrent(PowerStats &s) {
  if (s.metricTimeMs <= 0) return 0;
  return s.currentTimeSum / s.metricTimeMs;
}

float avgPower(PowerStats &s) {
  if (s.metricTimeMs <= 0) return 0;
  return s.powerTimeSum / s.metricTimeMs;
}

uint8_t* allocateImageBuffer(size_t bytes) {
  uint8_t *buf = nullptr;

  if (psramFound()) {
    buf = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }

  if (buf == nullptr) {
    buf = (uint8_t*)malloc(bytes);
  }

  return buf;
}

bool initImageBuffer() {
  aiInput = allocateImageBuffer(IMG_BYTES);

  if (aiInput == nullptr) {
    Serial.println("ERROR: Could not allocate AI image buffer.");
    return false;
  }

  Serial.print("AI buffer bytes: ");
  Serial.println(IMG_BYTES);
  return true;
}

// =======================
// Power sampling
// =======================
bool readPower() {
  busVoltage = ina219.getBusVoltage_V();
  shuntVoltage = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();
  loadVoltage = busVoltage + (shuntVoltage / 1000.0);

  power_mW = loadVoltage * current_mA;

  if (loadVoltage < 3.0 || loadVoltage > 6.0) return false;
  if (current_mA < 0 || current_mA > 1000) return false;
  if (power_mW < 0 || power_mW > 6000) return false;

  return true;
}

void samplePowerTo(PowerStats &s) {
  unsigned long now = millis();

  if (!readPower()) {
    return;
  }

  if (s.lastSampleMs == 0) {
    s.lastSampleMs = now;
    return;
  }

  unsigned long dt = now - s.lastSampleMs;
  s.lastSampleMs = now;

  if (dt == 0 || dt > 20000) {
    return;
  }

  if (current_mA > s.peakCurrent_mA) {
    s.peakCurrent_mA = current_mA;
  }

  if (power_mW > s.peakPower_mW) {
    s.peakPower_mW = power_mW;
  }

  if (loadVoltage < s.minLoadVoltage) {
    s.minLoadVoltage = loadVoltage;
  }

  s.currentTimeSum += current_mA * dt;
  s.powerTimeSum += power_mW * dt;
  s.metricTimeMs += dt;

  s.energy_mWh += power_mW * ((double)dt / 3600000.0);
}

// =======================
// Camera init
// =======================
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

  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = FRAMESIZE_96X96;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    Serial.println("PSRAM found.");
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    Serial.println("WARNING: PSRAM not found.");
    config.fb_location = CAMERA_FB_IN_DRAM;
    config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();

  if (s != NULL) {
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
  }

  return true;
}

// =======================
// Capture frame
// =======================
bool captureFrame() {
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed.");
    frameBytes = 0;
    return false;
  }

  unsigned long startUs = micros();

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, aiInput);

  captureUs = micros() - startUs;
  frameBytes = fb->len;

  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("RGB565 to RGB888 conversion failed.");
    return false;
  }

  return true;
}

// =======================
// Edge Impulse callback
// =======================
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;

  for (size_t i = 0; i < length; i++) {
    uint8_t r = aiInput[pixel_ix + 0];
    uint8_t g = aiInput[pixel_ix + 1];
    uint8_t b = aiInput[pixel_ix + 2];

    out_ptr[i] = (r << 16) + (g << 8) + b;

    pixel_ix += 3;
  }

  return 0;
}

// =======================
// Run AI
// =======================
bool runAI() {
  aiDecision = "ai_error";
  aiBestLabel = "none";
  personScore = 0.0;
  noPersonScore = 0.0;
  aiConfidence = 0.0;
  aiInferenceMs = 0;

  signal_t signal;
  signal.total_length = IMG_W * IMG_H;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  Serial.println("Running Edge Impulse AI inference. This may take about 8 seconds...");

  unsigned long startMs = millis();
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  aiInferenceMs = millis() - startMs;

  if (err != EI_IMPULSE_OK) {
    Serial.print("AI ERROR: run_classifier failed: ");
    Serial.println(err);
    aiDecision = "classifier_error";
    return false;
  }

  float bestScore = 0.0;
  const char *bestLabel = "unknown";

  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char *label = ei_classifier_inferencing_categories[i];
    float value = result.classification[i].value;

    if (strcmp(label, "person") == 0) {
      personScore = value;
    }

    if (strcmp(label, "no_person") == 0) {
      noPersonScore = value;
    }

    if (value > bestScore) {
      bestScore = value;
      bestLabel = label;
    }
  }

  aiBestLabel = String(bestLabel);
  aiConfidence = bestScore;

  if (personScore >= PERSON_THRESHOLD) {
    aiDecision = "PERSON_DETECTED";
  } else if (noPersonScore >= NO_PERSON_THRESHOLD) {
    aiDecision = "NO_PERSON";
  } else {
    aiDecision = "UNCERTAIN";
  }

  return true;
}

// =======================
// Low-power test sequence
// =======================
void runLowPowerMeasurement() {
  Serial.println();
  Serial.println("==== LOW-POWER MEASUREMENT START ====");
  Serial.println("Wi-Fi is OFF during idle and event measurement.");

  resetPowerStats(idleStats);
  resetPowerStats(eventStats);

  Serial.println("Measuring idle low-power mode for 30 seconds...");
  idleStartedAt = millis();

  while (millis() - idleStartedAt < idleMeasureMs) {
    samplePowerTo(idleStats);
    delay(100);
  }

  idleEndedAt = millis();

  Serial.println("Idle measurement done.");
  Serial.println("Waiting for PIR motion. Wave hand / walk in front of sensor.");

  int lastPir = digitalRead(PIR_PIN);

  while (true) {
    int pir = digitalRead(PIR_PIN);

    if (pir == HIGH && lastPir == LOW) {
      break;
    }

    lastPir = pir;
    delay(50);
  }

  Serial.println("PIR motion detected. Running event measurement.");

  eventStartedAt = millis();
  resetPowerStats(eventStats);

  samplePowerTo(eventStats);

  bool captured = captureFrame();

  if (!captured) {
    aiDecision = "capture_failed";
  } else {
    runAI();
  }

  unsigned long eventWindowStart = millis();

  while (millis() - eventWindowStart < eventMeasureMs) {
    samplePowerTo(eventStats);
    delay(100);
  }

  eventEndedAt = millis();

  testComplete = true;

  Serial.println("Low-power measurement complete.");
}

// =======================
// Web report
// =======================
String htmlReport() {
  String page = "";
  page += "<!DOCTYPE html><html><head>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>ESP32 Low-Power AI Report</title>";
  page += "<style>";
  page += "body{font-family:Arial;margin:25px;background:#111;color:#eee;}";
  page += ".card{background:#222;padding:18px;margin:12px 0;border-radius:12px;}";
  page += ".big{font-size:28px;font-weight:bold;}";
  page += "table{width:100%;border-collapse:collapse;}";
  page += "td,th{padding:8px;border-bottom:1px solid #444;text-align:left;}";
  page += ".good{color:#7CFC00;}.warn{color:#FFD700;}.bad{color:#FF6347;}";
  page += "button{padding:10px 15px;border:0;border-radius:8px;font-weight:bold;margin-right:8px;}";
  page += "</style></head><body>";

  page += "<h1>ESP32 Low-Power Edge AI Report</h1>";

  page += "<div class='card'>";
  page += "<h2>Test Status</h2>";
  page += "<p class='big good'>Complete</p>";
  page += "<p>Wi-Fi was OFF during idle and AI event measurement. Wi-Fi turned ON only afterward to display this report.</p>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Idle Low-Power Mode</h2>";
  page += "<table>";
  page += "<tr><td>Duration</td><td>" + String(idleEndedAt - idleStartedAt) + " ms</td></tr>";
  page += "<tr><td>Average Current</td><td>" + String(avgCurrent(idleStats), 3) + " mA</td></tr>";
  page += "<tr><td>Average Power</td><td>" + String(avgPower(idleStats), 3) + " mW</td></tr>";
  page += "<tr><td>Peak Current</td><td>" + String(idleStats.peakCurrent_mA, 3) + " mA</td></tr>";
  page += "<tr><td>Peak Power</td><td>" + String(idleStats.peakPower_mW, 3) + " mW</td></tr>";
  page += "<tr><td>Minimum Load Voltage</td><td>" + String((idleStats.minLoadVoltage == 999) ? 0 : idleStats.minLoadVoltage, 3) + " V</td></tr>";
  page += "<tr><td>Idle Energy</td><td>" + String(idleStats.energy_mWh, 6) + " mWh</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Motion AI Event</h2>";
  page += "<table>";
  page += "<tr><td>Duration</td><td>" + String(eventEndedAt - eventStartedAt) + " ms</td></tr>";
  page += "<tr><td>Frame Bytes</td><td>" + String(frameBytes) + "</td></tr>";
  page += "<tr><td>Capture Time</td><td>" + String(captureUs) + " us</td></tr>";
  page += "<tr><td>AI Decision</td><td class='big'>" + aiDecision + "</td></tr>";
  page += "<tr><td>Best Label</td><td>" + aiBestLabel + "</td></tr>";
  page += "<tr><td>Person Score</td><td>" + String(personScore * 100.0, 2) + " %</td></tr>";
  page += "<tr><td>No-Person Score</td><td>" + String(noPersonScore * 100.0, 2) + " %</td></tr>";
  page += "<tr><td>AI Confidence</td><td>" + String(aiConfidence * 100.0, 2) + " %</td></tr>";
  page += "<tr><td>Inference Time</td><td>" + String(aiInferenceMs) + " ms</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Event Power</h2>";
  page += "<table>";
  page += "<tr><td>Average Current</td><td>" + String(avgCurrent(eventStats), 3) + " mA</td></tr>";
  page += "<tr><td>Average Power</td><td>" + String(avgPower(eventStats), 3) + " mW</td></tr>";
  page += "<tr><td>Peak Current</td><td>" + String(eventStats.peakCurrent_mA, 3) + " mA</td></tr>";
  page += "<tr><td>Peak Power</td><td>" + String(eventStats.peakPower_mW, 3) + " mW</td></tr>";
  page += "<tr><td>Minimum Load Voltage</td><td>" + String((eventStats.minLoadVoltage == 999) ? 0 : eventStats.minLoadVoltage, 3) + " V</td></tr>";
  page += "<tr><td>Event Energy</td><td>" + String(eventStats.energy_mWh, 6) + " mWh</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<a href='/report.csv'><button>Download CSV Report</button></a>";
  page += "</body></html>";

  return page;
}

String csvReport() {
  String csv = "";

  csv += "mode,duration_ms,avg_current_mA,avg_power_mW,peak_current_mA,peak_power_mW,min_voltage_V,energy_mWh,ai_decision,best_label,person_score,no_person_score,ai_confidence,inference_ms,capture_us,frame_bytes\n";

  csv += "idle,";
  csv += String(idleEndedAt - idleStartedAt) + ",";
  csv += String(avgCurrent(idleStats), 3) + ",";
  csv += String(avgPower(idleStats), 3) + ",";
  csv += String(idleStats.peakCurrent_mA, 3) + ",";
  csv += String(idleStats.peakPower_mW, 3) + ",";
  csv += String((idleStats.minLoadVoltage == 999) ? 0 : idleStats.minLoadVoltage, 3) + ",";
  csv += String(idleStats.energy_mWh, 6) + ",";
  csv += ",,,,,,,\n";

  csv += "event,";
  csv += String(eventEndedAt - eventStartedAt) + ",";
  csv += String(avgCurrent(eventStats), 3) + ",";
  csv += String(avgPower(eventStats), 3) + ",";
  csv += String(eventStats.peakCurrent_mA, 3) + ",";
  csv += String(eventStats.peakPower_mW, 3) + ",";
  csv += String((eventStats.minLoadVoltage == 999) ? 0 : eventStats.minLoadVoltage, 3) + ",";
  csv += String(eventStats.energy_mWh, 6) + ",";
  csv += aiDecision + ",";
  csv += aiBestLabel + ",";
  csv += String(personScore, 5) + ",";
  csv += String(noPersonScore, 5) + ",";
  csv += String(aiConfidence, 5) + ",";
  csv += String(aiInferenceMs) + ",";
  csv += String(captureUs) + ",";
  csv += String(frameBytes) + "\n";

  return csv;
}

void handleRoot() {
  server.send(200, "text/html", htmlReport());
}

void handleCSV() {
  server.sendHeader("Content-Disposition", "attachment; filename=low_power_ai_report.csv");
  server.send(200, "text/csv", csvReport());
}

// =======================
// Turn Wi-Fi back on for report
// =======================
void startReportWiFi() {
  Serial.println();
  Serial.println("Turning Wi-Fi ON for report page...");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected.");

  if (MDNS.begin("edgecam-lowpower")) {
    Serial.println("mDNS started: http://edgecam-lowpower.local");
  }

  Serial.print("Open report IP in browser: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/report.csv", handleCSV);
  server.begin();

  wifiStarted = true;
}

// =======================
// Setup
// =======================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== LOW POWER TEST V2: WIFI-OFF MEASURE, WIFI-ON REPORT ====");

  pinMode(PIR_PIN, INPUT);

  if (!initImageBuffer()) {
    while (true) {
      delay(1000);
    }
  }

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

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  btStop();

  Serial.println("ESP32 Wi-Fi OFF for measurement.");
  Serial.println("Starting one-shot low-power test.");

  runLowPowerMeasurement();
  startReportWiFi();
}

// =======================
// Loop
// =======================
void loop() {
  if (wifiStarted) {
    server.handleClient();
  }

  delay(50);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif