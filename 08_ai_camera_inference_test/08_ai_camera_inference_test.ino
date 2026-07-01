#include <esp32_person_detector_v2_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =======================
// SunFounder new AI camera pin mapping
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
// Direct 96x96 camera input for Edge Impulse
// =======================
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  EI_CLASSIFIER_INPUT_WIDTH
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  EI_CLASSIFIER_INPUT_HEIGHT
#define EI_CAMERA_FRAME_BYTE_SIZE        3

#define CHANNEL_MODE 0
// 0 = RGB: c0,c1,c2
// 1 = BGR: c2,c1,c0

static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;

// =======================
// Function declarations
// =======================
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);
static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr);

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE IMPULSE CAMERA INFERENCE TEST ====");
  Serial.println("Library: esp32_person_detector_v2 TensorFlow float32");
  Serial.println("Camera: SunFounder ESP32 Camera Extension");
  Serial.println("Model: person vs no_person");

  Serial.print("EI input width: ");
  Serial.println(EI_CLASSIFIER_INPUT_WIDTH);
  Serial.print("EI input height: ");
  Serial.println(EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.print("EI label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

  if (ei_camera_init() == false) {
    Serial.println("Failed to initialize camera. Stopping.");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Camera initialized.");
  Serial.println("Starting inference in 2 seconds...");
  delay(2000);
}

void loop() {
  snapshot_buf = (uint8_t*)malloc(
    EI_CLASSIFIER_INPUT_WIDTH *
    EI_CLASSIFIER_INPUT_HEIGHT *
    EI_CAMERA_FRAME_BYTE_SIZE
  );

  if (snapshot_buf == nullptr) {
    Serial.println("ERROR: Failed to allocate snapshot buffer.");
    delay(5000);
    return;
  }

  unsigned long captureStart = millis();

  bool captured = ei_camera_capture(
    EI_CLASSIFIER_INPUT_WIDTH,
    EI_CLASSIFIER_INPUT_HEIGHT,
    snapshot_buf
  );

  unsigned long captureMs = millis() - captureStart;

  if (!captured) {
    Serial.println("ERROR: Failed to capture image.");
    free(snapshot_buf);
    snapshot_buf = nullptr;
    delay(5000);
    return;
  }

  uint32_t checksum = 0;
  uint32_t minVal = 255;
  uint32_t maxVal = 0;
  uint32_t sumVal = 0;

  int pixelBytes = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;

  for (int i = 0; i < pixelBytes; i++) {
    uint8_t v = snapshot_buf[i];
    checksum += v;
    sumVal += v;

    if (v < minVal) {
      minVal = v;
    }

    if (v > maxVal) {
      maxVal = v;
    }
  }

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  ei_impulse_result_t result = { 0 };

  Serial.println();
  Serial.println("================================");
  Serial.println("NEW CAMERA AI INFERENCE");
  Serial.print("Frame checksum: ");
  Serial.println(checksum);
  Serial.print("Input min/max/avg: ");
  Serial.print(minVal);
  Serial.print("/");
  Serial.print(maxVal);
  Serial.print("/");
  Serial.println((float)sumVal / pixelBytes, 2);
  Serial.print("Capture time ms: ");
  Serial.println(captureMs);
  Serial.println("Running classifier... wait about 8 seconds.");

  unsigned long inferenceStart = millis();
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  unsigned long inferenceMs = millis() - inferenceStart;

  if (err != EI_IMPULSE_OK) {
    Serial.print("ERROR: run_classifier failed: ");
    Serial.println(err);
    free(snapshot_buf);
    snapshot_buf = nullptr;
    delay(5000);
    return;
  }

  Serial.print("Inference time ms: ");
  Serial.println(inferenceMs);
  Serial.print("Timing DSP ms: ");
  Serial.println(result.timing.dsp);
  Serial.print("Timing classification ms: ");
  Serial.println(result.timing.classification);

  float bestScore = 0.0;
  const char *bestLabel = "unknown";

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  Serial.println("ERROR: This is an object detection model, but this project expects classification.");
#else
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    const char *label = ei_classifier_inferencing_categories[i];
    float value = result.classification[i].value;

    Serial.print(label);
    Serial.print(": ");
    Serial.println(value, 5);

    if (value > bestScore) {
      bestScore = value;
      bestLabel = label;
    }
  }

  Serial.print("BEST LABEL: ");
  Serial.println(bestLabel);
  Serial.print("CONFIDENCE: ");
  Serial.print(bestScore * 100.0, 2);
  Serial.println("%");
#endif

  Serial.println("================================");

  free(snapshot_buf);
  snapshot_buf = nullptr;

  delay(5000);
}

bool ei_camera_init(void) {
  if (is_initialised) {
    return true;
  }

  camera_config_t camera_config;

  camera_config.ledc_channel = LEDC_CHANNEL_0;
  camera_config.ledc_timer = LEDC_TIMER_0;

  camera_config.pin_d0 = Y2_GPIO_NUM;
  camera_config.pin_d1 = Y3_GPIO_NUM;
  camera_config.pin_d2 = Y4_GPIO_NUM;
  camera_config.pin_d3 = Y5_GPIO_NUM;
  camera_config.pin_d4 = Y6_GPIO_NUM;
  camera_config.pin_d5 = Y7_GPIO_NUM;
  camera_config.pin_d6 = Y8_GPIO_NUM;
  camera_config.pin_d7 = Y9_GPIO_NUM;

  camera_config.pin_xclk = XCLK_GPIO_NUM;
  camera_config.pin_pclk = PCLK_GPIO_NUM;
  camera_config.pin_vsync = VSYNC_GPIO_NUM;
  camera_config.pin_href = HREF_GPIO_NUM;

  camera_config.pin_sccb_sda = SIOD_GPIO_NUM;
  camera_config.pin_sccb_scl = SIOC_GPIO_NUM;

  camera_config.pin_pwdn = PWDN_GPIO_NUM;
  camera_config.pin_reset = RESET_GPIO_NUM;

  camera_config.xclk_freq_hz = 20000000;

  camera_config.pixel_format = PIXFORMAT_RGB565;
  camera_config.frame_size = FRAMESIZE_96X96;
  camera_config.jpeg_quality = 12;
  camera_config.fb_count = 1;

  if (psramFound()) {
    Serial.println("PSRAM found.");
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  } else {
    Serial.println("WARNING: PSRAM not found.");
    camera_config.fb_location = CAMERA_FB_IN_DRAM;
    camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  }

  esp_err_t err = esp_camera_init(&camera_config);

  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();

  if (s == NULL) {
    Serial.println("Could not get camera sensor.");
    return false;
  }

  s->set_vflip(s, 0);
  s->set_hmirror(s, 0);
  s->set_brightness(s, 0);
  s->set_saturation(s, 0);

  is_initialised = true;
  return true;
}

void ei_camera_deinit(void) {
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    Serial.println("Camera deinit failed.");
    return;
  }

  is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    Serial.println("ERROR: Camera is not initialized.");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    Serial.println("Camera capture failed.");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, out_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    Serial.println("RGB565 to RGB888 conversion failed.");
    return false;
  }

  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;

  for (size_t i = 0; i < length; i++) {
    uint8_t c0 = snapshot_buf[pixel_ix + 0];
    uint8_t c1 = snapshot_buf[pixel_ix + 1];
    uint8_t c2 = snapshot_buf[pixel_ix + 2];

#if CHANNEL_MODE == 0
    uint8_t r = c0;
    uint8_t g = c1;
    uint8_t b = c2;
#else
    uint8_t r = c2;
    uint8_t g = c1;
    uint8_t b = c0;
#endif

    out_ptr[i] = (r << 16) + (g << 8) + b;

    pixel_ix += 3;
  }

  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif