#ifndef CONTROL_H__
#define CONTROL_H__

#include "main.h"
int myabs(int a);
void Get_Angle(uint8_t way);
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
int Put_Down(float Angle,int encoder_left,int encoder_right);
int PWM_Limit(int IN,int max,int min);
#endif
