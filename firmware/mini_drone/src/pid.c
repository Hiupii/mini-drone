#include "../include/pid.h"

float pitch_current_error = 0;
float roll_current_error = 0;
float pitch_previous_error = 0;
float roll_previous_error = 0;
float Kp=2.5, Ki=0.05, Kd=0.01;
static float pitch_sum_error = 0;
static float roll_sum_error = 0;

float pitch_delta_error = 0;
float roll_delta_error = 0;

void pid_calculate()
{
  pitch_current_error = pitch_setpoint - pitch_input;
  roll_current_error = roll_setpoint - roll_input;
  pitch_sum_error += pitch_current_error;
  roll_sum_error += roll_current_error;
  pitch_delta_error = pitch_current_error - pitch_previous_error;
  roll_delta_error = roll_current_error - roll_previous_error;
  pitch_previous_error = pitch_current_error;
  roll_previous_error = roll_current_error;

  pitch_output = Kp * pitch_current_error + Ki * pitch_sum_error + Kd * pitch_delta_error;
  roll_output = Kp * roll_current_error + Ki * roll_sum_error + Kd * roll_delta_error;
}