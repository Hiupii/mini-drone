#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__

#ifdef __cplusplus
extern "C"
{
#endif

#define DIR_X         33
#define DIR_Y         32

#define HIGH_PIN      27
#define LOW_PIN       13
#define CAP_PIN       14

void joystick_setup(void);

void joystick_getdata(bool* btn1,
                      bool* btn2,
                      bool* btn3,
                      bool* btn4,
                      bool* btn5,
                      bool* btn6,
                      bool* btn7);

#ifdef __cplusplus
}
#endif

#endif //__JOYSTICK_H__