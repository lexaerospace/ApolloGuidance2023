#include "StateMachine.h"


StateMachine::checkState(){
  switch(state){
    case IDLE:
      break;
    case MOTOR_ACTIVE:
      break;
    case GUIDANCE_ACTIVE:
      break;
    case APOGEE:
      break;
    case DESCENT:
      break;
  }
}

StateMachine::startState(){
  state = IDLE;
}