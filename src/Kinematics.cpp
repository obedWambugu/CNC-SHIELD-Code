#include <Arduino.h>
#include "Config.h"
#include "Kinematics.h"
#include "Motors.h"

Axis a[NUM_AXES];

float theta1 = 0, d2 = 0, d3 = 0;
float px = 0, py = 0, pz = 0;

void forwardKinematics(float t1, float d2_val, float d3_val, float &x, float &y, float &z) {
    float theta_rad = t1 * PI / 180.0; // Convert to radians
    x = d3_val * cos(theta_rad);       // Radial distance in XY plane
    y = d3_val * sin(theta_rad);
    z = d2_val + BASE_HEIGHT;          // Vertical displacement
  }
  
bool inverseKinematics(float x, float y, float z, float &t1, float &d2_val, float &d3_val) {
    d3_val = sqrt(x * x + y * y);
    if (d3_val == 0) {
        t1 = 0;
    } else {
        t1 = atan2(y, x) * 180.0 / PI; // Returns -180° to 180°
        if (t1 < 0) t1 += 360.0;       // Normalize to 0°–360°
    }
    d2_val = z - BASE_HEIGHT;
    return true;
}

void position(float n_theta1, float n_d2, float n_d3) {
    theta1 = n_theta1;
    d2 = n_d2;
    d3 = n_d3;
    forwardKinematics(theta1, d2, d3, px, py, pz); // Update Cartesian position
  }

void line(float newx, float newy, float newz) {
    while(!isLimitSwitchTriggered){
        
    }
    float new_theta1, new_d2, new_d3;
    if (!inverseKinematics(newx, newy, newz, new_theta1, new_d2, new_d3)) {
        Serial.println(F("Invalid position"));
        return;
    }

    // Calculate steps for each axis
    a[0].delta = (new_theta1 - theta1) * STEPS_PER_DEGREE; // θ1 in degrees
    a[1].delta = (new_d2 - d2) * STEPS_PER_MM;             // d2 in mm
    a[2].delta = (new_d3 - d3) * STEPS_PER_MM;             // d3 in mm

    long maxsteps = 0;
    for (int i = 0; i < NUM_AXES; ++i) {
        a[i].absdelta = abs(a[i].delta);
        a[i].over = 0;
        if (maxsteps < a[i].absdelta) maxsteps = a[i].absdelta;
        digitalWrite(motors[i].dir_pin, a[i].delta > 0 ? HIGH : LOW);
    }

    long pulse_width = 5; // Microseconds
    long total_delay = step_delay;
    if (total_delay < pulse_width * 2) total_delay = pulse_width * 2;

    unsigned long start_time = micros();
    for (long i = 0; i < maxsteps; ++i) {
        for (int j = 0; j < NUM_AXES; ++j) {
        a[j].over += a[j].absdelta;
        if (a[j].over >= maxsteps) {
            a[j].over -= maxsteps;
            onestep(j);
        }
        }
        delayMicroseconds(total_delay - pulse_width);
    }
    unsigned long end_time = micros();
    Serial.print(F("Time (s): "));
    Serial.println((end_time - start_time) / 1000000.0);

    position(new_theta1, new_d2, new_d3); // Update position
}