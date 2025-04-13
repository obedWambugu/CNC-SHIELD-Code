#ifndef MOTORS_H
#define MOTORS_H

typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
} Motor;

extern Motor motors[NUM_AXES];

void motor_setup();
void motor_enable();
void motor_disable();
void onestep(int motor);
bool isLimitSwitchTriggered(int motor);
void homeAxis(int axis);

#endif