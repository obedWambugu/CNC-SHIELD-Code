
//------------------------------------------------------------------------------
// 3 Axis RPP Robot Demo - supports CNCShieldV3 on Arduino UNO
//------------------------------------------------------------------------------

#include <Arduino.h>
#include "Config.h"
#include "Motors.h"
#include "Kinematics.h"
#include "Gcode.h"

//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------

void setup() {
  Serial.begin(BAUD);
  delay(1000);
  Serial.flush();
  motor_setup();
  motor_enable();
  //homeAllAxes();
  position(0, 0, 0);
  feedrate(1000);    // Default feedrate in mm/min
  help();
  where();
  ready();
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);
    if (sofar < MAX_BUF - 1) buffer[sofar++] = c;
    if (c == '\n') {
      buffer[sofar] = 0;
      Serial.print(F("\r\n"));
      processCommand();
      ready();
    }
  }
}

int main() {
  init();
  setup();
  while (1) {
    loop();
  }
  return 0;
}