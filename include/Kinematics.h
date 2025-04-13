#ifndef KINEMATICS_H
#define KINEMATICS_H


typedef struct {
    long delta;       // Number of steps to move
    long absdelta;
    long over;        // For Bresenham calculations
  } Axis;

extern Axis a[NUM_AXES];

void forwardKinematics(float t1, float d2_val, float d3_val, float &x, float &y, float &z);
bool inverseKinematics(float x, float y, float z, float &t1, float &d2_val, float &d3_val);
void position(float n_theta1, float n_d2, float n_d3);
void line(float newx, float newy, float newz);

#endif