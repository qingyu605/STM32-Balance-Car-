#include "bluetooth.h"


enCarState g_newcarstate = enSTOP; //  1前2后3左4右0停止 1 forward 2 backward 3 left 4 right 0 stop
void bluetooth_init(void) {
    MX_UART5_Init(); // HAL库的UART5初始化（由CubeMX生成）
    // Init_PID(); // 保留
}

void UART5_Send_Byte(uint8_t data_byte) {
    // 等待发送寄存器为空（TXE标志），对应LL_USART_IsActiveFlag_TXE
    while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE) == RESET) {
    }

    // 发送一个字节，对应LL_USART_TransmitData8
    huart5.Instance->DR = data_byte;
}

void UART5_Send_String(uint8_t *data_str, uint16_t datasize) {
    // 直接调用HAL库的发送函数（内部已处理等待TXE标志的逻辑）
    HAL_UART_Transmit(&huart5, data_str, datasize, HAL_MAX_DELAY);
}

void UART5_Send_Char(char *s) {
    uint16_t len = strlen(s); // 计算字符串长度
    HAL_UART_Transmit(&huart5, (uint8_t *)s, len, HAL_MAX_DELAY);
}
