#include "../include/control.h"
#include <Arduino.h>

void control_set_point(bool btn1, bool btn2, bool btn3, bool btn4,
                       bool btn5, bool btn6, bool btn7)
{
  if (btn1 == 1) { //Up
    pitch_setpoint = 10;
  } else {
    pitch_setpoint = 0;
  }

  if (btn2 == 1) { //Down
    pitch_setpoint = -10;
  } else {
    pitch_setpoint = 0;
  }

  if (btn3 == 1) { //Left
    roll_setpoint = 10;
  } else {
    roll_setpoint = 0;
  }

  if (btn4 == 1) { //Right
    roll_setpoint = -10;
  } else {
    roll_setpoint = 0;
  }

  if (btn5 == 1) { //High
    base_throttle = HIGH_THROTTLE;
  } else {
    base_throttle = IDLE_THROTTLE;
  }

  if (btn6 == 1) { //Low
    base_throttle = LOW_THROTTLE;
  } else {
    base_throttle = IDLE_THROTTLE;
  }

  if (btn7 == 1) { //Cap

  } else {
    
  }
}

void prescale_output(float* ptr)
{
  *ptr = *ptr / 255 * 8191;
}