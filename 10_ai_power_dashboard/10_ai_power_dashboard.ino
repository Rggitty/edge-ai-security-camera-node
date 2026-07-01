#include <esp32_person_detector_v2_inferencing.h>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

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

const unsigned long eventEnergyWindowMs = 10000;

static bool debug_nn = false;

// Image buffers stored in PSRAM instead of internal DRAM
static uint8_t *aiInput = nullptr;
static uint8_t *latestRgb = nullptr;

bool latestRgbValid = false;
unsigned long latestImageVersion = 0;

String lastAiDecision = "not_run";
String lastAiBestLabel = "none";
float lastPersonScore = 0.0;
float lastNoPersonScore = 0.0;
float lastAiConfidence = 0.0;
unsigned long lastAiInferenceMs = 0;

bool aiBusy = false;

// =======================
// Live power values
// =======================
float busVoltage = 0;
float shuntVoltage = 0;
float loadVoltage = 0;
float current_mA = 0;
float power_mW = 0;

// =======================
// Power metrics
// =======================
float maxCurrent_mA = 0;
float maxPower_mW = 0;
float minLoadVoltage = 999;

double energy_mWh = 0;
double currentTimeSum = 0;
double powerTimeSum = 0;
double metricTimeMs = 0;

unsigned long lastPowerSampleMs = 0;

// =======================
// Event metrics
// =======================
unsigned long eventCount = 0;
unsigned long lastCaptureTime = 0;
const unsigned long cooldownMs = 5000;
int lastPirState = LOW;

unsigned long lastCaptureMicros = 0;
size_t lastImageBytes = 0;

bool eventWindowActive = false;
unsigned long eventWindowEndMs = 0;
double eventStartEnergy_mWh = 0;
double lastEventEnergy_mWh = 0;

String lastState = "booting";

// =======================
// CSV event logging
// =======================
const int MAX_EVENT_LOGS = 30;

struct EventRecord {
  unsigned long eventNumber;
  unsigned long timeMs;
  size_t imageBytes;
  unsigned long captureUs;
  double eventEnergy_mWh;
  float peakCurrent_mA;
  float peakPower_mW;
  float avgPower_mW;
  float minVoltage_V;

  String aiDecision;
  String aiBestLabel;
  float personScore;
  float noPersonScore;
  float aiConfidence;
  unsigned long aiInferenceMs;
};

EventRecord eventLogs[MAX_EVENT_LOGS];
int eventLogCount = 0;
int pendingEventIndex = -1;

// =======================
// Function declarations
// =======================
float averageCurrent();
float averagePower();
void finalizePendingEventLog();
bool captureFrameToAIInput();
bool runAIOnCurrentInput();
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

// =======================
// CSV helpers
// =======================
void addEventLogPlaceholder() {
  if (eventLogCount >= MAX_EVENT_LOGS) {
    for (int i = 1; i < MAX_EVENT_LOGS; i++) {
      eventLogs[i - 1] = eventLogs[i];
    }
    eventLogCount = MAX_EVENT_LOGS - 1;
  }

  pendingEventIndex = eventLogCount;

  eventLogs[eventLogCount].eventNumber = eventCount;
  eventLogs[eventLogCount].timeMs = millis();
  eventLogs[eventLogCount].imageBytes = lastImageBytes;
  eventLogs[eventLogCount].captureUs = lastCaptureMicros;
  eventLogs[eventLogCount].eventEnergy_mWh = 0;
  eventLogs[eventLogCount].peakCurrent_mA = maxCurrent_mA;
  eventLogs[eventLogCount].peakPower_mW = maxPower_mW;
  eventLogs[eventLogCount].avgPower_mW = averagePower();
  eventLogs[eventLogCount].minVoltage_V = (minLoadVoltage == 999) ? 0 : minLoadVoltage;

  eventLogs[eventLogCount].aiDecision = lastAiDecision;
  eventLogs[eventLogCount].aiBestLabel = lastAiBestLabel;
  eventLogs[eventLogCount].personScore = lastPersonScore;
  eventLogs[eventLogCount].noPersonScore = lastNoPersonScore;
  eventLogs[eventLogCount].aiConfidence = lastAiConfidence;
  eventLogs[eventLogCount].aiInferenceMs = lastAiInferenceMs;

  eventLogCount++;
}

void finalizePendingEventLog() {
  if (pendingEventIndex >= 0 && pendingEventIndex < eventLogCount) {
    eventLogs[pendingEventIndex].eventEnergy_mWh = lastEventEnergy_mWh;
    eventLogs[pendingEventIndex].peakCurrent_mA = maxCurrent_mA;
    eventLogs[pendingEventIndex].peakPower_mW = maxPower_mW;
    eventLogs[pendingEventIndex].avgPower_mW = averagePower();
    eventLogs[pendingEventIndex].minVoltage_V = (minLoadVoltage == 999) ? 0 : minLoadVoltage;
  }

  pendingEventIndex = -1;
}

void clearEventLog() {
  eventLogCount = 0;
  pendingEventIndex = -1;
}

// =======================
// Power sampling
// =======================
void samplePower() {
  unsigned long now = millis();

  busVoltage = ina219.getBusVoltage_V();
  shuntVoltage = ina219.getShuntVoltage_mV();
  current_mA = ina219.getCurrent_mA();
loadVoltage = busVoltage + (shuntVoltage / 1000.0);

power_mW = loadVoltage * current_mA;

if (loadVoltage < 3.0 || loadVoltage > 6.0 || current_mA < 0 || current_mA > 1000 || power_mW < 0 || power_mW > 6000) {
  return;
}
  if (lastPowerSampleMs == 0) {
    lastPowerSampleMs = now;
    return;
  }

  unsigned long dt = now - lastPowerSampleMs;
  lastPowerSampleMs = now;

  if (dt == 0 || dt > 20000) {
    return;
  }

  if (current_mA > maxCurrent_mA) {
    maxCurrent_mA = current_mA;
  }

  if (power_mW > maxPower_mW) {
    maxPower_mW = power_mW;
  }

  if (loadVoltage > 0.5 && loadVoltage < minLoadVoltage) {
    minLoadVoltage = loadVoltage;
  }

  currentTimeSum += current_mA * dt;
  powerTimeSum += power_mW * dt;
  metricTimeMs += dt;

  energy_mWh += power_mW * ((double)dt / 3600000.0);

  if (eventWindowActive && now >= eventWindowEndMs) {
    lastEventEnergy_mWh = energy_mWh - eventStartEnergy_mWh;
    eventWindowActive = false;
    lastState = "event_energy_done";
    finalizePendingEventLog();
  }
}

float averageCurrent() {
  if (metricTimeMs <= 0) return 0;
  return currentTimeSum / metricTimeMs;
}

float averagePower() {
  if (metricTimeMs <= 0) return 0;
  return powerTimeSum / metricTimeMs;
}

void resetMetrics() {
  maxCurrent_mA = 0;
  maxPower_mW = 0;
  minLoadVoltage = 999;

  energy_mWh = 0;
  currentTimeSum = 0;
  powerTimeSum = 0;
  metricTimeMs = 0;

  eventCount = 0;
  lastCaptureMicros = 0;
  lastImageBytes = 0;
  lastEventEnergy_mWh = 0;
  eventWindowActive = false;

  lastAiDecision = "not_run";
  lastAiBestLabel = "none";
  lastPersonScore = 0.0;
  lastNoPersonScore = 0.0;
  lastAiConfidence = 0.0;
  lastAiInferenceMs = 0;

  latestRgbValid = false;
  latestImageVersion = 0;

  lastState = "metrics_reset";
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
// Capture direct 96x96 RGB565 and convert to RGB888
// =======================
bool captureFrameToAIInput() {
  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed.");
    lastImageBytes = 0;
    return false;
  }

  unsigned long startMicros = micros();

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, aiInput);

  lastCaptureMicros = micros() - startMicros;
  lastImageBytes = fb->len;

  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("RGB565 to RGB888 conversion failed.");
    return false;
  }

  memcpy(latestRgb, aiInput, IMG_BYTES);
  latestRgbValid = true;
  latestImageVersion++;

  return true;
}

// =======================
// AI inference
// =======================
bool runAIOnCurrentInput() {
  aiBusy = true;

  lastAiDecision = "ai_running";
  lastAiBestLabel = "none";
  lastPersonScore = 0.0;
  lastNoPersonScore = 0.0;
  lastAiConfidence = 0.0;
  lastAiInferenceMs = 0;

  signal_t signal;
  signal.total_length = IMG_W * IMG_H;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  Serial.println("Running Edge Impulse AI inference. This may take about 8 seconds...");

  unsigned long inferenceStart = millis();
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  lastAiInferenceMs = millis() - inferenceStart;

  if (err != EI_IMPULSE_OK) {
    Serial.print("AI ERROR: run_classifier failed: ");
    Serial.println(err);

    lastAiDecision = "classifier_error";
    aiBusy = false;
    return false;
  }

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  lastAiDecision = "wrong_model_type";
  aiBusy = false;
  return false;
#else

  float bestScore = 0.0;
  const char *bestLabel = "unknown";

  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char *label = ei_classifier_inferencing_categories[i];
    float value = result.classification[i].value;

    if (strcmp(label, "person") == 0) {
      lastPersonScore = value;
    }

    if (strcmp(label, "no_person") == 0) {
      lastNoPersonScore = value;
    }

    if (value > bestScore) {
      bestScore = value;
      bestLabel = label;
    }
  }

  lastAiBestLabel = String(bestLabel);
  lastAiConfidence = bestScore;

  if (lastPersonScore >= PERSON_THRESHOLD) {
    lastAiDecision = "PERSON_DETECTED";
  } else if (lastNoPersonScore >= NO_PERSON_THRESHOLD) {
    lastAiDecision = "NO_PERSON";
  } else {
    lastAiDecision = "UNCERTAIN";
  }

  Serial.println("AI RESULT:");
  Serial.print("  no_person: ");
  Serial.println(lastNoPersonScore, 5);
  Serial.print("  person: ");
  Serial.println(lastPersonScore, 5);
  Serial.print("  decision: ");
  Serial.println(lastAiDecision);
  Serial.print("  confidence: ");
  Serial.println(lastAiConfidence * 100.0, 2);
  Serial.print("  inference ms: ");
  Serial.println(lastAiInferenceMs);

#endif

  aiBusy = false;
  return true;
}

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
// Capture event
// =======================
void captureImageEvent() {
  if (aiBusy) {
    lastState = "ai_busy";
    return;
  }

  eventCount++;
  lastState = "motion_capture";

  samplePower();

  eventWindowActive = true;
  eventStartEnergy_mWh = energy_mWh;
  eventWindowEndMs = millis() + eventEnergyWindowMs;

  bool captured = captureFrameToAIInput();

  if (!captured) {
    lastState = "capture_failed";
    lastAiDecision = "no_frame";
    addEventLogPlaceholder();
    return;
  }

  lastState = "image_saved_running_ai";

  bool aiOk = runAIOnCurrentInput();

  if (aiOk) {
    lastState = "capture_ok_ai_done";
  } else {
    lastState = "capture_ok_ai_failed";
  }

  addEventLogPlaceholder();

  samplePower();
}

// =======================
// BMP image sender
// =======================
void sendLatestBMP() {
  if (!latestRgbValid) {
    server.send(404, "text/plain", "No image captured yet.");
    return;
  }

  const int width = IMG_W;
  const int height = IMG_H;
  const int rowSize = ((width * 3 + 3) / 4) * 4;
  const int pixelDataSize = rowSize * height;
  const int fileSize = 54 + pixelDataSize;

  uint8_t header[54] = {0};

  header[0] = 'B';
  header[1] = 'M';

  header[2] = fileSize;
  header[3] = fileSize >> 8;
  header[4] = fileSize >> 16;
  header[5] = fileSize >> 24;

  header[10] = 54;
  header[14] = 40;

  header[18] = width;
  header[19] = width >> 8;
  header[20] = width >> 16;
  header[21] = width >> 24;

  header[22] = height;
  header[23] = height >> 8;
  header[24] = height >> 16;
  header[25] = height >> 24;

  header[26] = 1;
  header[28] = 24;

  header[34] = pixelDataSize;
  header[35] = pixelDataSize >> 8;
  header[36] = pixelDataSize >> 16;
  header[37] = pixelDataSize >> 24;

  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.setContentLength(fileSize);
  server.send(200, "image/bmp", "");

  WiFiClient client = server.client();
  client.write(header, 54);

  uint8_t row[rowSize];

  for (int y = height - 1; y >= 0; y--) {
    int rowIndex = 0;

    for (int x = 0; x < width; x++) {
      int idx = (y * width + x) * 3;

      uint8_t r = latestRgb[idx + 0];
      uint8_t g = latestRgb[idx + 1];
      uint8_t b = latestRgb[idx + 2];

      row[rowIndex++] = b;
      row[rowIndex++] = g;
      row[rowIndex++] = r;
    }

    while (rowIndex < rowSize) {
      row[rowIndex++] = 0;
    }

    client.write(row, rowSize);
  }
}

// =======================
// Webpage
// =======================
String htmlPage() {
  String page = "";
  page += "<!DOCTYPE html><html><head>";
  page += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  page += "<title>ESP32 Edge AI Power Dashboard</title>";
  page += "<style>";
  page += "body{font-family:Arial;margin:25px;background:#111;color:#eee;}";
  page += ".card{background:#222;padding:18px;margin:12px 0;border-radius:12px;}";
  page += ".big{font-size:30px;font-weight:bold;}";
  page += "table{width:100%;border-collapse:collapse;}";
  page += "td{padding:8px;border-bottom:1px solid #444;}";
  page += "button{padding:10px 15px;border:0;border-radius:8px;font-weight:bold;margin-right:8px;margin-bottom:8px;}";
  page += ".good{color:#7CFC00;}";
  page += ".warn{color:#FFD700;}";
  page += ".bad{color:#FF6347;}";
  page += "img{width:288px;height:288px;image-rendering:pixelated;border-radius:12px;border:2px solid #444;margin-top:10px;}";
  page += "</style>";

  page += "<script>";
  page += "let lastVersion=-1;";

  page += "async function update(){";
  page += "let r=await fetch('/data');";
  page += "let d=await r.json();";

  page += "document.getElementById('state').innerText=d.state;";
  page += "document.getElementById('events').innerText=d.events;";
  page += "document.getElementById('bus').innerText=d.bus_V.toFixed(3)+' V';";
  page += "document.getElementById('load').innerText=d.load_V.toFixed(3)+' V';";
  page += "document.getElementById('current').innerText=d.current_mA.toFixed(3)+' mA';";
  page += "document.getElementById('power').innerText=d.power_mW.toFixed(3)+' mW';";
  page += "document.getElementById('maxcurrent').innerText=d.max_current_mA.toFixed(3)+' mA';";
  page += "document.getElementById('maxpower').innerText=d.max_power_mW.toFixed(3)+' mW';";
  page += "document.getElementById('avgcurrent').innerText=d.avg_current_mA.toFixed(3)+' mA';";
  page += "document.getElementById('avgpower').innerText=d.avg_power_mW.toFixed(3)+' mW';";
  page += "document.getElementById('minvolt').innerText=d.min_load_V.toFixed(3)+' V';";
  page += "document.getElementById('energy').innerText=d.energy_mWh.toFixed(6)+' mWh';";
  page += "document.getElementById('eventenergy').innerText=d.last_event_energy_mWh.toFixed(6)+' mWh';";
  page += "document.getElementById('bytes').innerText=d.image_bytes;";
  page += "document.getElementById('capture').innerText=d.capture_us+' us';";
  page += "document.getElementById('uptime').innerText=d.time_ms+' ms';";
  page += "document.getElementById('version').innerText=d.image_version;";

  page += "document.getElementById('aidecision').innerText=d.ai_decision;";
  page += "document.getElementById('aibest').innerText=d.ai_best_label;";
  page += "document.getElementById('personscore').innerText=(d.person_score*100).toFixed(2)+' %';";
  page += "document.getElementById('nopersonscore').innerText=(d.no_person_score*100).toFixed(2)+' %';";
  page += "document.getElementById('aiconf').innerText=(d.ai_confidence*100).toFixed(2)+' %';";
  page += "document.getElementById('aitime').innerText=d.ai_inference_ms+' ms';";

  page += "let aiBox=document.getElementById('aidecision');";
  page += "aiBox.className='big ';";
  page += "if(d.ai_decision=='PERSON_DETECTED'){aiBox.className+='bad';}";
  page += "else if(d.ai_decision=='NO_PERSON'){aiBox.className+='good';}";
  page += "else{aiBox.className+='warn';}";

  page += "if(d.image_version!=lastVersion && d.image_version>0){";
  page += "lastVersion=d.image_version;";
  page += "document.getElementById('latestimg').src='/latest.bmp?t='+Date.now();";
  page += "document.getElementById('imgstatus').innerText='Latest 96x96 AI frame loaded';";
  page += "}";

  page += "}";

  page += "async function resetStats(){await fetch('/reset');update();}";
  page += "async function forceCapture(){document.getElementById('state').innerText='force_capture_running_ai_wait';await fetch('/capture');setTimeout(update,500);}";
  page += "async function clearLog(){await fetch('/clearlog');update();}";

  page += "setInterval(update,1000);window.onload=update;";
  page += "</script>";

  page += "</head><body>";
  page += "<h1>Edge AI Security Camera Node</h1>";

  page += "<div class='card'><div>State</div><div class='big good' id='state'>...</div></div>";

  page += "<div class='card'>";
  page += "<h2>AI Person Detection</h2>";
  page += "<table>";
  page += "<tr><td>AI Decision</td><td><span id='aidecision' class='big warn'>...</span></td></tr>";
  page += "<tr><td>Best Label</td><td id='aibest'>...</td></tr>";
  page += "<tr><td>Person Score</td><td id='personscore'>...</td></tr>";
  page += "<tr><td>No Person Score</td><td id='nopersonscore'>...</td></tr>";
  page += "<tr><td>AI Confidence</td><td id='aiconf'>...</td></tr>";
  page += "<tr><td>Inference Time</td><td id='aitime'>...</td></tr>";
  page += "</table>";
  page += "<p class='warn'>Thresholds: PERSON >= 80%, NO_PERSON >= 80%, otherwise UNCERTAIN.</p>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Latest AI Frame</h2>";
  page += "<p id='imgstatus'>No image captured yet. Move in front of the PIR sensor or press Force Capture.</p>";
  page += "<img id='latestimg' alt='Latest 96x96 AI frame will appear here'>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Live Power</h2>";
  page += "<table>";
  page += "<tr><td>Bus Voltage</td><td id='bus'>...</td></tr>";
  page += "<tr><td>Load Voltage</td><td id='load'>...</td></tr>";
  page += "<tr><td>Current</td><td id='current'>...</td></tr>";
  page += "<tr><td>Power</td><td id='power'>...</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Power Profiling Metrics</h2>";
  page += "<table>";
  page += "<tr><td>Peak Current</td><td id='maxcurrent'>...</td></tr>";
  page += "<tr><td>Peak Power</td><td id='maxpower'>...</td></tr>";
  page += "<tr><td>Average Current</td><td id='avgcurrent'>...</td></tr>";
  page += "<tr><td>Average Power</td><td id='avgpower'>...</td></tr>";
  page += "<tr><td>Minimum Load Voltage</td><td id='minvolt'>...</td></tr>";
  page += "<tr><td>Total Energy Since Reset</td><td id='energy'>...</td></tr>";
  page += "<tr><td>Last Event Energy, 10 sec AI window</td><td id='eventenergy'>...</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<div class='card'>";
  page += "<h2>Motion / Camera</h2>";
  page += "<table>";
  page += "<tr><td>Motion Events</td><td id='events'>...</td></tr>";
  page += "<tr><td>Last Frame Bytes</td><td id='bytes'>...</td></tr>";
  page += "<tr><td>Last Capture Time</td><td id='capture'>...</td></tr>";
  page += "<tr><td>Image Version</td><td id='version'>...</td></tr>";
  page += "<tr><td>Uptime</td><td id='uptime'>...</td></tr>";
  page += "</table>";
  page += "</div>";

  page += "<button onclick='resetStats()'>Reset Metrics + Log</button>";
  page += "<button onclick='forceCapture()'>Force Capture + AI</button>";
  page += "<a href='/eventlog.csv'><button>Download CSV</button></a>";
  page += "<button onclick='clearLog()'>Clear Event Log</button>";
  page += "<p class='warn'>Motion triggers camera capture, Edge Impulse AI inference, power profiling, and CSV logging.</p>";
  page += "</body></html>";

  return page;
}

// =======================
// Web handlers
// =======================
void handleRoot() {
  server.send(200, "text/html", htmlPage());
}

void handleLatestBMP() {
  sendLatestBMP();
}

void handleData() {
  samplePower();

  float minV = minLoadVoltage;
  if (minV == 999) {
    minV = 0;
  }

  String json = "{";
  json += "\"time_ms\":" + String(millis()) + ",";
  json += "\"state\":\"" + lastState + "\",";
  json += "\"events\":" + String(eventCount) + ",";
  json += "\"bus_V\":" + String(busVoltage, 3) + ",";
  json += "\"shunt_mV\":" + String(shuntVoltage, 3) + ",";
  json += "\"load_V\":" + String(loadVoltage, 3) + ",";
  json += "\"current_mA\":" + String(current_mA, 3) + ",";
  json += "\"power_mW\":" + String(power_mW, 3) + ",";
  json += "\"max_current_mA\":" + String(maxCurrent_mA, 3) + ",";
  json += "\"max_power_mW\":" + String(maxPower_mW, 3) + ",";
  json += "\"avg_current_mA\":" + String(averageCurrent(), 3) + ",";
  json += "\"avg_power_mW\":" + String(averagePower(), 3) + ",";
  json += "\"min_load_V\":" + String(minV, 3) + ",";
  json += "\"energy_mWh\":" + String(energy_mWh, 6) + ",";
  json += "\"last_event_energy_mWh\":" + String(lastEventEnergy_mWh, 6) + ",";
  json += "\"image_bytes\":" + String(lastImageBytes) + ",";
  json += "\"capture_us\":" + String(lastCaptureMicros) + ",";
  json += "\"image_version\":" + String(latestImageVersion) + ",";

  json += "\"ai_decision\":\"" + lastAiDecision + "\",";
  json += "\"ai_best_label\":\"" + lastAiBestLabel + "\",";
  json += "\"person_score\":" + String(lastPersonScore, 5) + ",";
  json += "\"no_person_score\":" + String(lastNoPersonScore, 5) + ",";
  json += "\"ai_confidence\":" + String(lastAiConfidence, 5) + ",";
  json += "\"ai_inference_ms\":" + String(lastAiInferenceMs);

  json += "}";

  server.send(200, "application/json", json);
}

void handleEventCSV() {
  String csv = "event,time_ms,image_bytes,capture_us,event_energy_mWh,peak_current_mA,peak_power_mW,avg_power_mW,min_voltage_V,ai_decision,ai_best_label,person_score,no_person_score,ai_confidence,ai_inference_ms\n";

  for (int i = 0; i < eventLogCount; i++) {
    csv += String(eventLogs[i].eventNumber) + ",";
    csv += String(eventLogs[i].timeMs) + ",";
    csv += String(eventLogs[i].imageBytes) + ",";
    csv += String(eventLogs[i].captureUs) + ",";
    csv += String(eventLogs[i].eventEnergy_mWh, 6) + ",";
    csv += String(eventLogs[i].peakCurrent_mA, 3) + ",";
    csv += String(eventLogs[i].peakPower_mW, 3) + ",";
    csv += String(eventLogs[i].avgPower_mW, 3) + ",";
    csv += String(eventLogs[i].minVoltage_V, 3) + ",";

    csv += eventLogs[i].aiDecision + ",";
    csv += eventLogs[i].aiBestLabel + ",";
    csv += String(eventLogs[i].personScore, 5) + ",";
    csv += String(eventLogs[i].noPersonScore, 5) + ",";
    csv += String(eventLogs[i].aiConfidence, 5) + ",";
    csv += String(eventLogs[i].aiInferenceMs) + "\n";
  }

  server.sendHeader("Content-Disposition", "attachment; filename=edge_ai_power_log.csv");
  server.send(200, "text/csv", csv);
}

void handleClearEventLog() {
  clearEventLog();
  server.send(200, "text/plain", "event log cleared");
}

void handleReset() {
  resetMetrics();
  clearEventLog();
  server.send(200, "text/plain", "metrics and event log reset");
}

void handleForceCapture() {
  captureImageEvent();
  server.send(200, "text/plain", "forced capture + AI complete");
}

// =======================
// Setup and loop
// =======================

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

bool initImageBuffers() {
  aiInput = allocateImageBuffer(IMG_BYTES);
  latestRgb = allocateImageBuffer(IMG_BYTES);

  if (aiInput == nullptr || latestRgb == nullptr) {
    Serial.println("ERROR: Could not allocate AI image buffers.");

    if (aiInput != nullptr) {
      free(aiInput);
      aiInput = nullptr;
    }

    if (latestRgb != nullptr) {
      free(latestRgb);
      latestRgb = nullptr;
    }

    return false;
  }

  Serial.print("AI buffer bytes: ");
  Serial.println(IMG_BYTES);
  Serial.println("AI image buffers allocated.");
  return true;
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE AI CAMERA + POWER DASHBOARD V2 ====");
  Serial.println("Model: esp32_person_detector_v2 TensorFlow float32");
  Serial.println("Camera mode: 96x96 RGB565 direct input");

  if (!initImageBuffers()) {
  while (true) {
    delay(1000);
  }
}

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

  if (MDNS.begin("edgecam")) {
    Serial.println("mDNS started: http://edgecam.local");
  }

  Serial.print("Open this IP in your browser: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/latest.bmp", handleLatestBMP);
  server.on("/reset", handleReset);
  server.on("/capture", handleForceCapture);
  server.on("/eventlog.csv", handleEventCSV);
  server.on("/clearlog", handleClearEventLog);

  server.begin();

  lastState = "ready";
  samplePower();

  Serial.println("Dashboard ready.");
}

void loop() {
  server.handleClient();

  samplePower();

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

  if (lastState == "ready") {
    lastState = "idle_waiting";
  }

  delay(100);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif