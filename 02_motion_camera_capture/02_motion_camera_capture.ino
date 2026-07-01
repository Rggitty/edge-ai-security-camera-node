#include "esp_camera.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =======================
// PIR motion sensor
// =======================
#define PIR_PIN 4

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

void captureImageEvent() {
  eventCount++;

  Serial.println();
  Serial.println("================================");
  Serial.print("MOTION EVENT #");
  Serial.println(eventCount);
  Serial.println("Capturing image...");

  unsigned long startTime = millis();

  camera_fb_t *fb = esp_camera_fb_get();

  unsigned long captureTime = millis() - startTime;

  if (!fb) {
    Serial.println("CAPTURE FAILED");
    Serial.println("================================");
    return;
  }

  Serial.println("CAPTURE OK");
  Serial.print("Image bytes: ");
  Serial.println(fb->len);
  Serial.print("Capture time ms: ");
  Serial.println(captureTime);
  Serial.print("Timestamp ms: ");
  Serial.println(millis());

  esp_camera_fb_return(fb);

  Serial.println("Frame returned to memory.");
  Serial.println("================================");
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE AI SECURITY CAMERA NODE - STAGE 1 ====");
  Serial.println("Motion-triggered camera capture.");
  Serial.println("No INA219 power profiling yet.");

  pinMode(PIR_PIN, INPUT);

  Serial.println("Initializing camera...");

  if (!initCamera()) {
    Serial.println("STOPPED: Camera failed.");
    while (true) {
      delay(1000);
    }
  }

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
    }
  }

  if (pirState == LOW && lastPirState == HIGH) {
    Serial.println("Motion ended.");
  }

  lastPirState = pirState;
  delay(100);
}