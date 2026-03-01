#ifndef DELAY_H__
#define DELAY_H__

#include "stm32f1xx_hal.h"
void delay_us_nop(uint16_t us);
void delay_us(uint32_t nus);
void delay_ms(uint16_t nms);
void delay_init(void);
#endif
