#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Kinematics.h"

Motor motors[NUM_AXES];

void motor_setup() {
    motors[0].step_pin = 2;   // θ1
    motors[0].dir_pin = 5;
    motors[0].enable_pin = 8;
    motors[0].limit_switch_pin = 9;
  
    motors[1].step_pin = 3;   // d2
    motors[1].dir_pin = 6;
    motors[1].enable_pin = 8;
    motors[1].limit_switch_pin = 10;
  
    motors[2].step_pin = 4;   // d3
    motors[2].dir_pin = 7;
    motors[2].enable_pin = 8;
    motors[2].limit_switch_pin = 11;
  
    for (int i = 0; i < NUM_AXES; ++i) {
      pinMode(motors[i].step_pin, OUTPUT);
      pinMode(motors[i].dir_pin, OUTPUT);
      pinMode(motors[i].enable_pin, OUTPUT);
      pinMode(motors[i].limit_switch_pin, INPUT_PULLUP);
    }
  }

void motor_enable() {
    for (int i = 0; i < NUM_AXES; ++i) {
        digitalWrite(motors[i].enable_pin, LOW);
    }
}

void motor_disable() {
    for (int i = 0; i < NUM_AXES; ++i) {
        digitalWrite(motors[i].enable_pin, HIGH);
    }
}

void onestep(int motor) {
    digitalWrite(motors[motor].step_pin, HIGH);
    digitalWrite(motors[motor].step_pin, LOW);
}

// Robust limit switch check with debouncing
bool isLimitSwitchTriggered(int motor) {
    if (digitalRead(motors[motor].limit_switch_pin) == LOW) {
        delay(DEBOUNCE_DELAY); // Wait to debounce
    if (digitalRead(motors[motor].limit_switch_pin) == LOW) {
        return true; // Confirmed trigger
     }
    }
    return false;
}
  
// Home a single axis
void homeAxis(int axis) {
    Serial.print(F("Homing axis "));
    Serial.println(axis);

    if (axis == 2){
        digitalWrite(motors[axis].dir_pin, HIGH);
    } else {
        // Set direction toward home (assume negative direction for simplicity)
        digitalWrite(motors[axis].dir_pin, LOW); // Adjust based on your setup
    }

    // Move toward limit switch at homing speed
    long steps_per_sec = HOMING_SPEED * (axis == 0 ? STEPS_PER_DEGREE : STEPS_PER_MM) / 60.0;
    // long step_delay = 1000000L / steps_per_sec;
    long step_delay = 1000000L / (steps_per_sec * 2);
    
    while (!isLimitSwitchTriggered(axis)) {
        onestep(axis);
        delayMicroseconds(step_delay);
    }
       
    if (axis == 2){
        digitalWrite(motors[axis].dir_pin, LOW);
    } else {
        // Back off slightly after hitting the limit
        digitalWrite(motors[axis].dir_pin, HIGH); // Reverse direction
    }

    long backoff_steps = HOMING_BACKOFF * (axis == 0 ? STEPS_PER_DEGREE : STEPS_PER_MM);
    if (axis == 1){
        backoff_steps = backoff_steps * 40;
    }
    else{
        backoff_steps = backoff_steps;
    }
    for (long i = 0; i < backoff_steps; i++) {
        onestep(axis);
        delayMicroseconds(step_delay);
    }

    // Set this position as zero
    if (axis == 0) position(0, d2, d3);
    else if (axis == 1) position(theta1, 0, d3);
    else position(theta1, d2, 400);

    Serial.print(F("Axis "));
    Serial.print(axis);
    Serial.println(F(" homed"));
}