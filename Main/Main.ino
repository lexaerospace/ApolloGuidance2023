#include "Sensors.h"

Sensors* sensors = new Sensors();
enum stateMachine {START, IDLE, MOTOR_ACTIVE, GUIDANCE_ACTIVE, APOGEE, DESCENT};
stateMachine state;
float startAlt = 0, currentAlt = 0;
unsigned long startTime = 0, previousTime = 0, currentTime = 0;
float velocity;
float thrust;
void setup() {
  Serial.begin(9600);
  sensors->debugMPU();
  sensors->setupMPU();
  sensors->debugBMP();
  sensors->setupBMP();
  state = IDLE;
  delay(100);
}

void loop() {
  currentTime = micros();
  // sensors->getAltitude();
  sensors_vec_t acceleration = sensors->getAcceleration();
  sensors_vec_t gyro = sensors->getGyro();
  //Serial.print(acceleration.x);
  // Serial.print(", "); Serial.print(acceleration.y);
  // Serial.print(", "); Serial.println(acceleration.z);
  // Serial.print(gyro.x);
  // Serial.print(", "); Serial.print(gyro.y);
  // Serial.print(", "); Serial.println(gyro.z);
  sensors->getAngle(acceleration, gyro);
  Serial.print(" , "); Serial.print(-180); Serial.print(" , "); Serial.print(180); Serial.print(" , ");
  currentAlt = sensors->getAltitude();
  //Serial.print(128); Serial.print(" , ");
  //Serial.print(129); Serial.print(" , ");
  Serial.println(currentAlt);
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
      if(currentAlt - startTime >= 1000){
        state = GUIDANCE_ACTIVE;
      }
      break;
    case GUIDANCE_ACTIVE:
      thrust = FThrust(velocity, currentAlt);
      break;
    case APOGEE:
      break;
    case DESCENT:
      break;
  }
}

float FThrust(float v, float hi) {
  float m = 421.5;
  float g = 9.8;
  float hf = 249.94;
  float vf = 0;
  float p = 1.225; // air density
  float A = 57.951; // reference area
  float CDrag = 0.7028363376417; // drag coefficient
  float F = m * v*v - m * g - (p * v*v * CDrag * A) / 2;
  return F;
}

