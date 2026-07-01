#define PIR_PIN 33

int lastState = LOW;
unsigned long eventCount = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_PIN, INPUT);

  Serial.println();
  Serial.println("PIR Motion Sensor Test");
  Serial.println("Wait 30 seconds for PIR warm-up, then move in front of it.");
}

void loop() {
  int currentState = digitalRead(PIR_PIN);

  if (currentState == HIGH && lastState == LOW) {
    eventCount++;
    Serial.print("MOTION DETECTED, event #");
    Serial.println(eventCount);
  }

  if (currentState == LOW && lastState == HIGH) {
    Serial.println("Motion ended");
  }

  lastState = currentState;
  delay(100);
}