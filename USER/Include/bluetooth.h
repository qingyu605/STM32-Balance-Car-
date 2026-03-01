#ifndef BLUETOOTH_H__
#define BLUETOOTH_H__

#include "main.h"

void bluetooth_init(void);
void UART5_Send_Byte(uint8_t data_byte);
void UART5_Send_String(uint8_t *data_str, uint16_t datasize);
void UART5_Send_Char(char *s);

#endif
