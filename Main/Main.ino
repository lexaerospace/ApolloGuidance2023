#include "Sensors.h"

Sensors* sensors = new Sensors();

void setup() {
  Serial.begin(9600);
  sensors->debugMPU();
  sensors->setupMPU();
  sensors->debugBMP();
  sensors->setupBMP();
  delay(100);
}

void loop() {
  //sensors->getAccel();
  //sensors->getGyro();
  Serial.println(sensors->getAltitude());
  delay(100);
}
