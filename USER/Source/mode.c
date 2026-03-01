#include "mode.h"
//
//
//
// 与编译器左侧行数对应
// 创建与枚举顺序完全对应的字符串数组（标签名称）
extern uint8_t usart5_txbuf[64]; // 用于usart5发送缓冲区，k210串口
extern uint8_t usart5_rxbuf[64]; // 用于usart5接收缓冲区，k210串口
char *CarModeToString(uint8_t mode_id) {
    // 字符串数组顺序必须与枚举成员顺序一致
    static char *modeNames[] = {
        "<1>Normal",          // 正常模式,包含蓝牙控制
        "<2>U_Follow",        // 超声波跟随
        "<3>U_Avoid",         // 超声波避障
        "<4>Weight_M",        // 负重模式
        "<5>PS2_Control",     // ps2控制
        "<6>Line_Track",      // 4路巡线
        "<7>Diff_Line_track", // 高难度4路巡线
        "<8>K210_QR",         // k210识别二维码
        "<9>K210_Line",       // k210巡线
        "<10>K210_Follow",     // k210跟随
        "<11>K210_SelfLearn",  // k210自主学习 需要KPU模型
        "<12>K210_mnist",      // k210识别数字  需要KPU模型
        "<13>K210_Img",        // k210图像识别 需要KPU模型
        "<14>LiDar_avoid",     // 雷达避障
        "<15>LiDar_Follow",    // 雷达跟随
        "<16>LiDar_aralm",     // 雷达警卫
        "<17>LiDar_Patrol",    // 雷达巡逻
        "<18>LiDar_Line",      // 雷达巡墙边做直线
        "<19>LiDar_wall_Line", // 雷达沿着墙边走
        "<20>CCD_Mode",        // ccd巡线
        "<21>ElE_Mode"         // 电磁巡线
        // 注意：Mode_Max 是边界值，不需要添加到数组中
    };
    // 范围检查：如果数值超出有效枚举范围，返回"Invalid mode"
    if (mode_id >= Mode_Max) {
        return "Invalid mode";
    }
    // 直接通过数值（索引）返回对应的标签字符串
    return modeNames[mode_id];
}
void select_mode(uint8_t mode) {
    switch (mode) {
    case 0: // 正常模式,包含蓝牙控制
			Car_Target_Velocity=0;
			Car_Turn_Amplitude_speed=0;
			Mid_Angle = -5;
        break;
    case 1: // 超声波跟随
        break;
    case 2: // 超声波避障
        break;
    case 3: // 负重模式
        break;
    case 4: // ps2控制
        break;
    case 5: // 4路巡线
        break;
    case 6: // 高难度4路巡线
        break;
    case 7: // k210识别二维码
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 8: // k210巡线
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 9: // k210跟随
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 10: // k210自主学习 需要KPU模型
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 11: // k210识别数字  需要KPU模型
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 12: // k210图像识别 需要KPU模型
        HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
        break;
    case 13: // 雷达避障
        break;
    case 14: // 雷达跟随
        break;
    case 15: // 雷达警卫
        break;
    case 16: // 雷达巡逻
        break;
    case 17: // 雷达巡墙边做直线
        break;
    case 18: // 雷达沿着墙边走
        break;
    case 19: // ccd巡线
        break;
    case 20: // 电磁巡线
        break;
    }
}
