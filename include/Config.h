#ifndef CONFIG_H
#define CONFIG_H

//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define VERSION              (3)                      // Firmware version
#define BAUD                 (115200)                 // Arduino baud rate
#define MAX_BUF              (64)                     // Serial buffer size
#define STEPS_PER_TURN       (200)                    // Steps per revolution for stepper motors
#define STEPS_PER_TURN_2     (200) 
#define STEPS_PER_DEGREE     (STEPS_PER_TURN_2 / 22.5) // For θ1 (rotational axis)
#define STEPS_PER_MM         (STEPS_PER_TURN)  // For d2, d3 (prismatic axes), adjust as needed
#define MAX_FEEDRATE         (1000000)                // Max steps per second
#define MIN_FEEDRATE         (1)                      // Min steps per second
#define NUM_AXES             (3)                      // 3 axes: θ1, d2, d3

// Robot geometry (adjust based on your specific RPP design)
#define BASE_HEIGHT          (0.0)                    // Base height offset (mm), if any

// Homing parameters
#define HOMING_SPEED         (500)                    // Feedrate for homing (mm/min)
#define HOMING_BACKOFF       (5)                      // Distance to back off after hitting limit (mm or degrees)
#define DEBOUNCE_DELAY       (10)                     // Debounce time for limit switches (ms)

// Jogging parameters
#define JOG_SPEED            (500)                    // Default jogging feedrate (mm/min or deg/min)
#define JOG_STEP_DEFAULT     (1.0)                    // Default jog step size (mm or degrees)

// Global variables
extern char buffer[MAX_BUF];
extern int sofar;
extern float fr;
extern long step_delay;
extern float theta1, d2, d3;
extern float px, py, pz;
extern char mode_abs;
extern long line_number;

#endif