#include <Rockit-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// =======================
// SunFounder / AI Thinker-style OV2640 camera pin mapping
// RESET is 33 for your SunFounder camera extension board
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
// Edge Impulse camera constants
// Direct 96x96 capture test
// =======================
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS  EI_CLASSIFIER_INPUT_WIDTH
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS  EI_CLASSIFIER_INPUT_HEIGHT
#define EI_CAMERA_FRAME_BYTE_SIZE        3

static bool debug_nn = false;
static bool is_initialised = false;
static uint8_t *snapshot_buf = nullptr;

// This changes how the same image bytes are packed into Edge Impulse.
int channelMode = 0;

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
  Serial.println("==== EDGE IMPULSE ESP32 CAMERA CHANNEL TEST ====");
  Serial.println("SunFounder ESP32 Camera Extension");
  Serial.println("Model: person vs no_person");
  Serial.print("EI input width: ");
  Serial.println(EI_CLASSIFIER_INPUT_WIDTH);
  Serial.print("EI input height: ");
  Serial.println(EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.print("EI label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
    while (true) {
      delay(1000);
    }
  } else {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting channel-order inference test in 2 seconds...\n");
  ei_sleep(2000);
}

void loop() {
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  snapshot_buf = (uint8_t*)malloc(
    EI_CLASSIFIER_INPUT_WIDTH *
    EI_CLASSIFIER_INPUT_HEIGHT *
    EI_CAMERA_FRAME_BYTE_SIZE
  );

  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  unsigned long captureStart = millis();

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH,
                        (size_t)EI_CLASSIFIER_INPUT_HEIGHT,
                        snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    snapshot_buf = nullptr;
    return;
  }

  unsigned long captureMs = millis() - captureStart;

  // Check the actual 96x96 model input buffer
  uint32_t checksum = 0;
  uint32_t minVal = 255;
  uint32_t maxVal = 0;
  uint32_t sumVal = 0;

  int pixelBytes = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT * 3;

  for (int i = 0; i < pixelBytes; i++) {
    uint8_t v = snapshot_buf[i];
    checksum += v;
    sumVal += v;
    if (v < minVal) minVal = v;
    if (v > maxVal) maxVal = v;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  const char* modeNames[] = {
    "RGB c0,c1,c2",
    "BGR c2,c1,c0",
    "GRB c1,c0,c2",
    "GBR c1,c2,c0",
    "RBG c0,c2,c1",
    "BRG c2,c0,c1"
  };

  ei_printf("\n================================\n");
  ei_printf("NEW CAMERA FRAME\n");
  ei_printf("Frame checksum: %lu\n", checksum);
  ei_printf("Input min/max/avg: %lu/%lu/%.2f\n",
            minVal,
            maxVal,
            (float)sumVal / pixelBytes);
  ei_printf("Capture time: %lu ms\n", captureMs);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  ei_printf("ERROR: This is an object detection model, but this project expects classification.\n");
#else

  for (int m = 0; m < 6; m++) {
    channelMode = m;

    ei_impulse_result_t result = { 0 };

    unsigned long inferenceStart = millis();
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    unsigned long inferenceMs = millis() - inferenceStart;

    if (err != EI_IMPULSE_OK) {
      ei_printf("\nMODE %d failed. Error: %d\n", m, err);
      continue;
    }

    ei_printf("\nMODE %d: %s\n", m, modeNames[m]);
    ei_printf("Inference time: %lu ms\n", inferenceMs);
    ei_printf("Timing DSP: %d ms\n", result.timing.dsp);
    ei_printf("Timing classification: %d ms\n", result.timing.classification);
    ei_printf("Predictions:\n");

    float bestScore = 0.0;
    const char *bestLabel = "unknown";

    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
      const char *label = ei_classifier_inferencing_categories[i];
      float value = result.classification[i].value;

      ei_printf("  %s: %.5f\r\n", label, value);

      if (value > bestScore) {
        bestScore = value;
        bestLabel = label;
      }
    }

    ei_printf("BEST LABEL: %s\n", bestLabel);
    ei_printf("CONFIDENCE: %.2f%%\n", bestScore * 100.0);
  }

#endif

#if EI_CLASSIFIER_HAS_ANOMALY
  ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

  ei_printf("================================\n");

  free(snapshot_buf);
  snapshot_buf = nullptr;
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

  // Direct 96x96 RGB565 capture avoids JPEG resize path for this test.
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
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
}

bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, out_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("RGB565 to RGB888 conversion failed\n");
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

    uint8_t r, g, b;

    switch (channelMode) {
      case 0: // RGB
        r = c0; g = c1; b = c2;
        break;

      case 1: // BGR
        r = c2; g = c1; b = c0;
        break;

      case 2: // GRB
        r = c1; g = c0; b = c2;
        break;

      case 3: // GBR
        r = c1; g = c2; b = c0;
        break;

      case 4: // RBG
        r = c0; g = c2; b = c1;
        break;

      case 5: // BRG
        r = c2; g = c0; b = c1;
        break;

      default:
        r = c0; g = c1; b = c2;
        break;
    }

    out_ptr[i] = (r << 16) + (g << 8) + b;
    pixel_ix += 3;
  }

  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif