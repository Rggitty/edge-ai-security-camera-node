#include <Wire.h>
#include <Adafruit_INA219.h>

#define I2C_SDA 13
#define I2C_SCL 14

Adafruit_INA219 ina219;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("==== INA219 I2C TEST ====");

  Wire.begin(I2C_SDA, I2C_SCL);

  if (!ina219.begin(&Wire)) {
    Serial.println("ERROR: INA219 not found.");
    Serial.println("Check VCC, GND, SDA, SCL wiring.");
    while (true) {
      delay(1000);
    }
  }

  ina219.setCalibration_32V_2A();

  Serial.println("INA219 found successfully.");
  Serial.println("time_ms,bus_V,shunt_mV,current_mA,power_mW");
}

void loop() {
  float busVoltage = ina219.getBusVoltage_V();
  float shuntVoltage = ina219.getShuntVoltage_mV();
  float current_mA = ina219.getCurrent_mA();
  float power_mW = ina219.getPower_mW();

  Serial.print(millis());
  Serial.print(",");
  Serial.print(busVoltage, 3);
  Serial.print(",");
  Serial.print(shuntVoltage, 3);
  Serial.print(",");
  Serial.print(current_mA, 3);
  Serial.print(",");
  Serial.println(power_mW, 3);

  delay(500);
}