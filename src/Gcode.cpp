#include <Arduino.h>
#include "Config.h"
#include "GCode.h"
#include "Kinematics.h"
#include "Motors.h"

char buffer[MAX_BUF];
int sofar = 0;
float fr = 0;
long step_delay = 0;
char mode_abs = 1;
long line_number = 0;

void pause(long ms) {
    delay(ms / 1000);
    delayMicroseconds(ms % 1000);
  }

void feedrate(float nfr) {
    float steps_per_sec = nfr * STEPS_PER_MM / 60.0; // Feedrate in mm/min to steps/sec
    if (fr == steps_per_sec) return;
    if (steps_per_sec > MAX_FEEDRATE || steps_per_sec < MIN_FEEDRATE) {
      Serial.print(F("New feedrate must be between "));
      Serial.print(MIN_FEEDRATE * 60.0 / STEPS_PER_MM);
      Serial.print(F(" and "));
      Serial.print(MAX_FEEDRATE * 60.0 / STEPS_PER_MM);
      Serial.println(F(" mm/min."));
      return;
    }
    step_delay = 1000000.0 / steps_per_sec;
    fr = steps_per_sec;
}

void cannedCycleSquare() {
    Serial.println(F("Starting C01 Square Canned Cycle"));

    // Define square parameters (20mm x 20mm square at z=10mm)
    float start_x = 0;
    float start_y = 0;
    float z_height = 10;
    float side_length = 20;

    // Ensure motors are enabled
    motor_enable();

    // Move to starting position
    line(start_x, start_y, z_height);
    Serial.println(F("Moved to start position"));
    where();

    // Perform square pattern
    // Point 1 to 2: Move right
    line(start_x + side_length, start_y, z_height);
    Serial.println(F("Square point 2"));
    where();

    // Point 2 to 3: Move up
    line(start_x + side_length, start_y + side_length, z_height);
    Serial.println(F("Square point 3"));
    where();

    // Point 3 to 4: Move left
    line(start_x, start_y + side_length, z_height);
    Serial.println(F("Square point 4"));
    where();

    // Point 4 to 1: Move down (return to start)
    line(start_x, start_y, z_height);
    Serial.println(F("Returned to start"));
    where();

    Serial.println(F("C01 Square Canned Cycle Complete"));
}

  
float parseNumber(char code, float val) {
    char *ptr = buffer;
    while ((long)ptr > 1 && (*ptr) && (long)ptr < (long)buffer + sofar) {
        if (*ptr == code) {
        return atof(ptr + 1);
        }
        ptr = strchr(ptr, ' ') + 1;
    }
    return val;
}

void output(char *code, float val) {
    Serial.print(code);
    Serial.print(val);
    Serial.print(" ");
}

void arc(float x_end, float y_end, float i, float j, bool clockwise) {
    // Current position (px, py) is the start point
    float center_x = px + i;  // Arc center X
    float center_y = py + j;  // Arc center Y
    float start_x = px;
    float start_y = py;
    float end_x = mode_abs ? x_end : px + x_end;
    float end_y = mode_abs ? y_end : py + y_end;
  
    // Calculate radius from start point to center
    float dx = start_x - center_x;
    float dy = start_y - center_y;
    float radius = sqrt(dx * dx + dy * dy);
  
    // Calculate start and end angles (in radians)
    float start_angle = atan2(dy, dx);
    float end_dx = end_x - center_x;
    float end_dy = end_y - center_y;
    float end_angle = atan2(end_dy, end_dx);
  
    // Ensure angles are in the correct direction
    if (clockwise) {
      if (end_angle > start_angle) end_angle -= 2 * PI;
    } else {
      if (end_angle < start_angle) end_angle += 2 * PI;
    }
  
    // Number of segments for smooth arc (adjustable)
    const int segments = 36;  // 10-degree steps
    float angle_step = (end_angle - start_angle) / segments;
  
    // Generate arc points
    for (int k = 1; k <= segments; k++) {
      float angle = start_angle + angle_step * k;
      float next_x = center_x + radius * cos(angle);
      float next_y = center_y + radius * sin(angle);
      line(next_x, next_y, pz);  // Move to next point at current Z
    }
    // Ensure endpoint is exact
    line(end_x, end_y, pz);
}
  
void where() {
    output("X ", px);
    output("Y ", py);
    output("Z ", pz);
    output("T ", theta1);
    output("D2 ", d2);
    output("D3 ", d3);
    output("F ", fr / STEPS_PER_MM * 60);
    Serial.println(mode_abs ? "ABS" : "REL");
}
  
void help() {
    Serial.print(F("RPP_Robot_Demo "));
    Serial.println(VERSION);
    Serial.println(F("Commands:"));
    Serial.println(F("G00/G01 [X/Y/Z(mm)] [F(feedrate)]; - linear move"));
    Serial.println(F("G02 [X/Y(mm)] [I/J(mm)] [F]; - clockwise arc"));
    Serial.println(F("G03 [X/Y(mm)] [I/J(mm)] [F]; - counterclockwise arc"));
    Serial.println(F("G04 P[seconds]; - delay"));
    Serial.println(F("G28; - home all axes"));
    Serial.println(F("G90; - absolute mode"));
    Serial.println(F("G91; - relative mode"));
    Serial.println(F("G92 [X/Y/Z(mm)]; - change logical position"));
    Serial.println(F("J1 [D(degrees)] [F(deg/min)]; - jog θ1 (degrees)"));
    Serial.println(F("J2/J3 [D(distance)]; - d2 (mm), d3 (mm)"));
    Serial.println(F("M17; - enable motors"));
    Serial.println(F("M18; - disable motors"));
    Serial.println(F("M100; - this help message"));
    Serial.println(F("M114; - report position and feedrate"));
    Serial.println(F("C01; - run square pattern canned cycle"));
}

void homeAllAxes() {
    Serial.println(F("Starting homing sequence"));
    motor_enable();
    for (int i = 0; i < NUM_AXES; i++) {
      homeAxis(i);
    }
    Serial.println(F("Homing complete"));
    where();
}

void jogTheta1(float degrees, float feedrate_deg_min) {
  Serial.print(F("Jogging theta1 by "));
  Serial.print(degrees);
  Serial.print(F(" degrees at "));
  Serial.print(feedrate_deg_min);
  Serial.println(F(" deg/min"));

  motor_enable();

  // Validate feedrate (assuming MAX_FEEDRATE is in mm/min, scale for degrees)
  float max_feedrate_deg = MAX_FEEDRATE * STEPS_PER_MM / STEPS_PER_DEGREE;
  float min_feedrate_deg = MIN_FEEDRATE * STEPS_PER_MM / STEPS_PER_DEGREE;
  if (feedrate_deg_min < min_feedrate_deg || feedrate_deg_min > max_feedrate_deg) {
      Serial.print(F("Error: Feedrate must be between "));
      Serial.print(min_feedrate_deg);
      Serial.print(F(" and "));
      Serial.print(max_feedrate_deg);
      Serial.println(F(" deg/min"));
      ready();
      return;
  }

  float new_theta1 = theta1 + degrees;

  long steps = (long)(degrees * STEPS_PER_DEGREE);
  if (steps == 0) {
      Serial.println(F("No movement required"));
      ready();
      return;
  }

  // Set direction
  digitalWrite(motors[0].dir_pin, steps >= 0 ? HIGH : LOW);
  steps = abs(steps);

  // Calculate step delay (degrees/min to microseconds/step)
  float steps_per_sec = feedrate_deg_min * STEPS_PER_DEGREE / 60.0;
  long step_delay_us = 1000000L / steps_per_sec;

  Serial.print(F("Theta1 steps: "));
  Serial.print(steps);
  Serial.print(F(", delay: "));
  Serial.print(step_delay_us);
  Serial.println(F(" us"));

  // Step motor
  for (long i = 0; i < steps; i++) {
      onestep(0);
      delayMicroseconds(step_delay_us);
  }

  theta1 = new_theta1;
  forwardKinematics(theta1, d2, d3, px, py, pz);

  Serial.print(F("New theta1: "));
  Serial.println(theta1, 2);
  where();
  ready();
}

void jogAxis(int axis, float distance, float feedrate_mm_min) {
  Serial.print(F("Jogging axis "));
  Serial.print(axis == 1 ? "d2" : "d3");
  Serial.print(F(" by "));
  Serial.print(distance);
  Serial.print(F(" mm at "));
  Serial.print(feedrate_mm_min);
  Serial.println(F(" mm/min"));

  motor_enable();

  // Validate feedrate
  if (feedrate_mm_min < MIN_FEEDRATE || feedrate_mm_min > MAX_FEEDRATE) {
      Serial.print(F("Error: Feedrate must be between "));
      Serial.print(MIN_FEEDRATE);
      Serial.print(F(" and "));
      Serial.print(MAX_FEEDRATE);
      Serial.println(F(" mm/min"));
      ready();
      return;
  }

  float new_d2 = d2;
  float new_d3 = d3;
  float prev_theta1 = theta1;

  if (axis == 1) {
      new_d2 += distance;
      if (new_d2 < 0.0 || new_d2 > 1000.0) {
          Serial.println(F("Error: d2 out of bounds"));
          ready();
          return;
      }
  } else if (axis == 2) {
      new_d3 += distance;
      if (new_d3 < 0.0 || new_d3 > 1000.0) {
          Serial.println(F("Error: d3 out of bounds"));
          ready();
          return;
      }
  } else {
      Serial.println(F("Error: Invalid axis"));
      ready();
      return;
  }

  float prev_fr = fr;
  feedrate(feedrate_mm_min);  // Set feedrate for line()

  float new_x, new_y, new_z;
  forwardKinematics(prev_theta1, new_d2, new_d3, new_x, new_y, new_z);
  line(new_x, new_y, new_z);

  feedrate(prev_fr);

  d2 = new_d2;
  d3 = new_d3;
  theta1 = prev_theta1;

  forwardKinematics(theta1, d2, d3, px, py, pz);

  Serial.print(F("After jog: theta1="));
  Serial.print(theta1, 2);
  Serial.print(F(", d2="));
  Serial.print(d2, 2);
  Serial.print(F(", d3="));
  Serial.println(d3, 2);
  where();
  ready();
}
  
void processCommand() {
    int cmd = parseNumber('G', -1);
    switch (cmd) {
      case 0:
      case 1: {
        feedrate(parseNumber('F', fr));
        line(parseNumber('X', mode_abs ? px : 0) + (mode_abs ? 0 : px),
             parseNumber('Y', mode_abs ? py : 0) + (mode_abs ? 0 : py),
             parseNumber('Z', mode_abs ? pz : 0) + (mode_abs ? 0 : pz));
        ready();
        break;
      }
      case 2: {
        feedrate(parseNumber('F', fr));
        arc(parseNumber('X', px), parseNumber('Y', py),
            parseNumber('I', 0), parseNumber('J', 0), true);
        ready();
        break;
      }
      case 3: {
        feedrate(parseNumber('F', fr));
        arc(parseNumber('X', px), parseNumber('Y', py),
            parseNumber('I', 0), parseNumber('J', 0), false);
        ready();
        break;
      }
      case 4: pause(parseNumber('P', 0) * 1000); ready(); break;
      case 28: homeAllAxes(); ready(); break;
      case 90: mode_abs = 1; ready(); break;
      case 91: mode_abs = 0; ready(); break;
      case 92: {
        float x = parseNumber('X', 0);
        float y = parseNumber('Y', 0);
        float z = parseNumber('Z', 0);
        float t1, d2_val, d3_val;
        inverseKinematics(x, y, z, t1, d2_val, d3_val);
        position(t1, d2_val, d3_val);
        ready();
        break;
      }
      case -1: {
        cmd = parseNumber('M', -1);
        switch (cmd) {
          case 17: motor_enable(); ready(); break;
          case 18: motor_disable(); ready(); break;
          case 100: help(); ready(); break;
          case 114: where(); ready(); break;
        }
        cmd = parseNumber('C', -1);
        switch (cmd) {
          case 1: cannedCycleSquare(); ready(); break;
        }
        cmd = parseNumber('J', -1);
        switch (cmd) {
          case 1: {
            float degrees = parseNumber('D', JOG_STEP_DEFAULT);
            float feedrate = parseNumber('F', JOG_SPEED);  // Default to JOG_SPEED
            jogTheta1(degrees, feedrate);
            break;
           }
           case 2: {
            float distance = parseNumber('D', JOG_STEP_DEFAULT);
            float feedrate = parseNumber('F', JOG_SPEED);
            jogAxis(1, distance, feedrate);
            break;
        }
        case 3: {
            float distance = parseNumber('D', JOG_STEP_DEFAULT);
            float feedrate = parseNumber('F', JOG_SPEED);
            jogAxis(2, distance, feedrate);
            break;
        }
        }
        break;
      }
    }
  }
  
void ready() {
    sofar = 0;
    Serial.print(F("ready>"));
    Serial.flush();
}

