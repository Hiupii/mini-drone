#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C"
{
#endif
#include <stdbool.h>

extern float pitch_setpoint;
extern float roll_setpoint;
extern float base_throttle;

#define MOT1            15
#define MOT2            14
#define MOT3            2
#define MOT4            4

#define CHN_1           0
#define CHN_2           1
#define CHN_3           2
#define CHN_4           3

#define FREQ            5000
#define RES             13

#define IDLE_THROTTLE   4915
#define HIGH_THROTTLE   5400
#define LOW_THROTTLE    4500

void control_set_point(bool btn1, bool btn2, bool btn3, bool btn4,
                       bool btn5, bool btn6, bool btn7);
void prescale_output(float* ptr);

#ifdef __cplusplus
}
#endif
#endif