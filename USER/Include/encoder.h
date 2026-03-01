#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"
#include "stm32f1xx_hal.h"
short encoder_left_getval(void);
short encoder_right_getval(void);
int Read_Encoder(int num);
#endif
