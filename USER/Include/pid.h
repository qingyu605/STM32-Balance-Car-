#ifndef PID_H__
#define PID_H__

#include "main.h"
#define DefaultSpeed 0
void position_control_speed(int xms);  // 位置式PID控制函数
void increment_control_speed(int xms); // 增量式PID控制函数
int Balance_PD(float Angle,float Gyro);
int Velocity_PI(int encoder_left,int encoder_right);
int Turn_PD(float gyro);

extern int Ut_left;                    // PID控制器的输出
extern int Ut_right;                   // PID控制器的输出
//前进速度
//Forward speed
extern float Car_Target_Velocity; //0-10
//旋转速度
//Rotation speed
extern float Car_Turn_Amplitude_speed; //0-60
#endif
