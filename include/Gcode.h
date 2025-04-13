#ifndef GCODE_H
#define GCODE_H

extern char buffer[MAX_BUF];
extern int sofar;

void feedrate(float nfr);
void processCommand();
void help();
void where();
void ready();
void cannedCycleSquare();
void homeAllAxes();
void jogAxis(int axis, float distance);

#endif