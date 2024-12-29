#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" 
{
#endif
float pitch_setpoint, pitch_input, pitch_output;
float roll_setpoint, roll_input, roll_output;

void pid_calculate();

#ifdef __cplusplus
}
#endif
#endif