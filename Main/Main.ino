#include "Sensors.h"

Sensors* sensors = new Sensors();

void setup() {
  Serial.begin(115200);
  sensors->debugMPU();
  sensors->setupMPU();
  sensors->debugBMP();
  sensors->setupBMP();
  delay(100);
}

void loop() {
  sensors->getAccel();
  sensors->getGyro();
  sensors->getAltitude();
  delay(100);
}
