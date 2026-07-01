#include <esp32_person_detector_v2_inferencing.h>

static bool debug_nn = false;
int syntheticMode = 0;

static int synthetic_get_data(size_t offset, size_t length, float *out_ptr) {
  for (size_t i = 0; i < length; i++) {
    size_t pixelIndex = offset + i;

    int x = pixelIndex % EI_CLASSIFIER_INPUT_WIDTH;
    int y = pixelIndex / EI_CLASSIFIER_INPUT_WIDTH;

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    switch (syntheticMode) {
      case 0:
        // black image
        r = 0; g = 0; b = 0;
        break;

      case 1:
        // white image
        r = 255; g = 255; b = 255;
        break;

      case 2:
        // gray image
        r = 128; g = 128; b = 128;
        break;

      case 3:
        // horizontal red gradient
        r = map(x, 0, EI_CLASSIFIER_INPUT_WIDTH - 1, 0, 255);
        g = 0;
        b = 0;
        break;

      case 4:
        // green/blue checkerboard
        if (((x / 8) + (y / 8)) % 2 == 0) {
          r = 0; g = 255; b = 0;
        } else {
          r = 0; g = 0; b = 255;
        }
        break;

      case 5:
        // simple fake room-ish beige/gray
        r = 150 + ((x + y) % 40);
        g = 140 + ((x * 2) % 30);
        b = 120 + ((y * 2) % 30);
        break;
    }

    out_ptr[i] = (r << 16) + (g << 8) + b;
  }

  return 0;
}

void runSyntheticTest(int mode, const char *name) {
  syntheticMode = mode;

  signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &synthetic_get_data;

  ei_impulse_result_t result = { 0 };

  Serial.println();
  Serial.println("================================");
  Serial.print("SYNTHETIC MODE ");
  Serial.print(mode);
  Serial.print(": ");
  Serial.println(name);

  unsigned long startMs = millis();
  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  unsigned long inferenceMs = millis() - startMs;

  if (err != EI_IMPULSE_OK) {
    Serial.print("ERROR: run_classifier failed: ");
    Serial.println(err);
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
  Serial.println("================================");
}

void setup() {
  Serial.begin(115200);
  delay(2000);

  Serial.println();
  Serial.println("==== EDGE IMPULSE SYNTHETIC INPUT TEST ====");
  Serial.println("No camera used. This checks if the Arduino model reacts to different inputs.");
  Serial.print("Input width: ");
  Serial.println(EI_CLASSIFIER_INPUT_WIDTH);
  Serial.print("Input height: ");
  Serial.println(EI_CLASSIFIER_INPUT_HEIGHT);
  Serial.print("Label count: ");
  Serial.println(EI_CLASSIFIER_LABEL_COUNT);

  runSyntheticTest(0, "black");
  runSyntheticTest(1, "white");
  runSyntheticTest(2, "gray");
  runSyntheticTest(3, "red gradient");
  runSyntheticTest(4, "green/blue checkerboard");
  runSyntheticTest(5, "fake room-ish colors");

  Serial.println();
  Serial.println("Synthetic test complete.");
}

void loop() {
  delay(1000);
}