#include "encoder.h"

// 编码器值获取
short encoder_left_getval(void) {
    short val = TIM3->CNT;
    TIM3->CNT = 0;
    return val;
}
short encoder_right_getval(void) {
    short val = TIM4->CNT;
    TIM4->CNT = 0;
    return (-1 * val);
}
/**************************************************************************
Function: Read encoder count per unit time
Input   : TIMX：Timer
Output  : none
函数功能：单位时间读取编码器计数
入口参数：TIMX：定时器
返回  值：速度值
**************************************************************************/
int Read_Encoder(int num)
{
   int Encoder_TIM;    
   switch(num)
	 {
		 case MOTOR_ID_ML:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case MOTOR_ID_MR:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		 default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
