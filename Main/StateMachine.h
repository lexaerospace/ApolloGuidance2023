
class StateMachine{
  public:
    enum stateMachine {IDLE, MOTOR_ACTIVE, GUIDANCE_ACTIVE, APOGEE, DESCENT};
    stateMachine state;
    checkState();
    startState();
};