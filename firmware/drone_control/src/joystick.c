#include <Arduino.h>
#include "joystick.h"

void joystick_setup(void)
{
  pinMode(HIGH_PIN, INPUT_PULLUP);
  pinMode(LOW_PIN, INPUT_PULLUP);
  pinMode(CAP_PIN, INPUT_PULLUP);

  pinMode(DIR_X, INPUT);
  pinMode(DIR_Y, INPUT);
}

void joystick_getdata(bool* btn1,
                      bool* btn2,
                      bool* btn3,
                      bool* btn4,
                      bool* btn5,
                      bool* btn6,
                      bool* btn7)
{
  *btn5 = digitalRead(HIGH_PIN);
  *btn6 = digitalRead(LOW_PIN);
  *btn7 = digitalRead(CAP_PIN);

  int dir_X = analogRead(DIR_X);
  delayMicroseconds(10);
  int dir_Y = analogRead(DIR_Y);

  if (dir_X > 900) {
    *btn4 = 0;
    *btn3 = 1;
  } else  if (dir_X < 100) {
    *btn4 = 1;
    *btn3 = 0;
  } else {
    *btn4 = 1;
    *btn3 = 1;
  }

  if (dir_Y > 900) {
    *btn1 = 0;
    *btn2 = 1;
  } else  if (dir_Y < 100) {
    *btn1 = 1;
    *btn2 = 0;
  } else {
    *btn1 = 1;
    *btn2 = 1;
  }
}