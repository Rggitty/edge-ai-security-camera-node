#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <Wire.h>
#include <Adafruit_INA219.h>

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
// Event tracking
// =======================
unsigned long eventCount = 0;
unsigned long lastCaptureTime = 0;
const unsigned long cooldownMs = 5000;

int lastPirState = LOW;

// =======================
// Power logging function
// =======================
void logPower(const char* stateName) {
  float busVoltage = ina219.getBusVoltage_V();
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();
  float loadVoltage = busVoltage + (shuntVoltage / 1000.0);

  Serial.print("POWER,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(stateName);
  Serial.print(",");
  Serial.print(busVoltage, 3);
  Serial.print(",");
  Serial.print(shuntVoltage, 3);
  Serial.print(",");
  Serial.print(loadVoltage, 3);
  Serial.print(",");
  Serial.print(current_mA, 3);
  Serial.print(",");
  Serial.println(power_mW, 3);
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
  config.pixel_format = PIXFORMAT_JPEG;

  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if (psramFound()) {
    Serial.println("PSRAM found.");
    config.fb_location = CAMERA_FB_IN_PSRAM;
    config.grab_mode = CAMERA_GRAB_LATEST;
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

  Serial.println("Camera initialized successfully.");
  return true;
}

// =======================
// Capture event
// =======================
void captureImageEvent() {
  eventCount++;

  Serial.println();
  Serial.println("================================");
  Serial.print("MOTION EVENT #");
  Serial.println(eventCount);

  logPower("motion_detected");
  Serial.println("Capturing image...");
  logPower("before_capture");

  unsigned long startMicros = micros();

  camera_fb_t *fb = esp_camera_fb_get();

  unsigned long captureMicros = micros() - startMicros;

  if (!fb) {
    Serial.println("CAPTURE FAILED");
    logPower("capture_failed");
    Serial.println("================================");
    return;
  }

  logPower("after_capture");

  Serial.println("CAPTURE OK");
  Serial.print("Image bytes: ");
  Serial.println(fb->len);
  Serial.print("Capture time us: ");
  Serial.println(captureMicros);
  Serial.print("Capture time ms: ");
  Serial.println(captureMicros / 1000.0, 3);
  Serial.print("Timestamp ms: ");
  Serial.println(millis());

  esp_camera_fb_return(fb);

  logPower("after_frame_return");

  Serial.println("Frame returned to memory.");
  Serial.println("================================");
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE AI SECURITY CAMERA NODE - STAGE 2 ====");
  Serial.println("Motion-triggered camera capture + INA219 logger.");
  Serial.println("NOTE: Current/power values are not real until VIN+ and VIN- are in the power path.");

  pinMode(PIR_PIN, INPUT);

  Serial.println("Initializing INA219...");
  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina219.begin(&Wire)) {
    Serial.println("ERROR: INA219 not found.");
    Serial.println("Check VCC, GND, SDA, SCL wiring.");
    while (true) {
      delay(1000);
    }
  }

  ina219.setCalibration_32V_2A();

  Serial.println("INA219 initialized successfully.");
  Serial.println("CSV format:");
  Serial.println("POWER,time_ms,state,bus_V,shunt_mV,load_V,current_mA,power_mW");

  logPower("ina219_ready");

  Serial.println("Initializing camera...");

  if (!initCamera()) {
    Serial.println("STOPPED: Camera failed.");
    while (true) {
      logPower("camera_failed");
      delay(1000);
    }
  }

  logPower("camera_ready");

  Serial.println("System ready.");
  Serial.println("Wait 30 seconds for PIR warm-up, then move in front of it.");
}

void loop() {
  int pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH && lastPirState == LOW) {
    if (millis() - lastCaptureTime > cooldownMs) {
      lastCaptureTime = millis();
      captureImageEvent();
    } else {
      Serial.println("Motion ignored during cooldown.");
      logPower("cooldown_ignore");
    }
  }

  if (pirState == LOW && lastPirState == HIGH) {
    Serial.println("Motion ended.");
    logPower("motion_ended");
  }

  lastPirState = pirState;

  // Occasional idle power log
  static unsigned long lastIdleLog = 0;
  if (millis() - lastIdleLog > 5000) {
    lastIdleLog = millis();
    logPower("idle_waiting");
  }

  delay(100);
}