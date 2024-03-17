#include "Sensors.h"
#include "Motors.h"

Sensors* sensors = new Sensors();
Motors* motors = new Motors();
enum stateMachine {START, IDLE, MOTOR_ACTIVE, GUIDANCE_ACTIVE, APOGEE, DESCENT};
stateMachine state;
float startAlt = 0, currentAlt = 0;
unsigned long startTime = 0, previousTime = 0, currentTime = 0;
float currentVelocity;
float thrust;
void setup() {
  Serial.begin(9600);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  sensors->startMPU();
  sensors->setupBMP();
  sensors->calibrateMPU(2000);
  sensors->setupKalmanFilter2D();
  motors->calibrateMotors();
  state = IDLE;
}

void loop() {
  previousTime = currentTime;
  currentTime = micros();
  sensors->prepareMeasurements();
  currentAlt = sensors->getAltitude();
  currentVelocity = sensors->getVelocity();
  //Serial.print(currentAlt); Serial.print(", "); Serial.println(currentVelocity); Serial.println(motors->FThrust(currentVelocity, currentAlt));
  float thrust = motors->FThrust(70, 100);
  //Serial.println(thrust);
  motors->runMotors(1200);
  //fThrust = FThrust(acceleration, currentAlt);

}

void stateMachine(){
  switch(state){
    case START:
      //Can't use setup() because start altitutdes get screwed up
      startAlt = sensors->getAltitude();
      state = IDLE;
      break;
    case IDLE:
      if(currentAlt - startAlt >= 1){
        startTime = currentTime;
        state = MOTOR_ACTIVE;
      }
      break;
    case MOTOR_ACTIVE:
      //Rocket burn time is 1000ms
      if(currentTime - startTime >= 1000){
        state = GUIDANCE_ACTIVE;
      }
      break;
    case GUIDANCE_ACTIVE:
      thrust = motors->FThrust(currentVelocity, currentAlt);
      break;
    case APOGEE:
      break;
    case DESCENT:
      break;
  }
}

