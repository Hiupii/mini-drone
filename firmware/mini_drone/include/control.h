#ifndef __CONTROL_H__
#define __CONTROL_H__

#ifdef __cplusplus
extern "C"
{
#endif

extern float pitch_setpoint, pitch_input, pitch_output;
extern float roll_setpoint, roll_input, roll_output;

#define MOT1      15
#define MOT2      14
#define MOT3      2
#define MOT4      4

#define CHN_1   0
#define CHN_2   1
#define CHN_3   2
#define CHN_4   3

#define FREQ    5000
#define RES     13

void prescale_output(float* ptr);

#ifdef __cplusplus
}
#endif
#endif