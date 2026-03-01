/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* 用户包含部分 - 在此处添加包含文件 */

/* 用户包含结束 */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 私有类型定义部分 - 在此处添加结构体、枚举等类型定义 */

/* 私有类型定义结束 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 私有宏定义部分 - 在此处添加宏定义 */

/* 私有宏定义结束 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* 私有宏函数部分 - 在此处添加宏函数 */

/* 私有宏函数结束 */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* 私有变量部分 - 在此处声明全局变量 */
//
uint8_t GET_Angle_Way = 2; // 获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波  //Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
float Angle_Balance, Gyro_Balance, Gyro_Turn; // 平衡倾角 平衡陀螺仪 转向陀螺仪 //Balance tilt angle balance gyroscope steering gyroscope
int Motor_Left, Motor_Right;                  // 电机PWM变量 //Motor PWM variable
int Temperature;                              // 温度变量 		//Temperature variable
float Acceleration_Z;                         // Z轴加速度计  //Z-axis accelerometer
int Mid_Angle;                                // 机械中值  //Mechanical median
float Move_X, Move_Z;                         // Move_X:前进速度  Move_Z：转向速度  //Move_X: Forward speed Move_Z: Steering speed
u8 Stop_Flag = 1;                             // 0:开始 1:停止  //0: Start 1: Stop
// pid控制相关变量
int Encoder_Left, Encoder_Right;         // 左右编码器的脉冲计数 Pulse counting of left and right encoders
int Balance_Pwm, Velocity_Pwm, Turn_Pwm; // 平衡环PWM变量，速度环PWM变量，转向环PWM变 Balance loop PWM variable, speed loop PWM variable, steering loop PWM variable
extern int curSpeed_left;                // 左轮当前速度(rpm)
extern int curSpeed_right;               // 右轮当前速度(rpm)
extern int goalSpeed_left;               // 左轮电机的目标速度(rpm)
extern int goalSpeed_right;              // 左轮电机的目标速度(rpm)
// pid模式
int pid_mode = 3; // PID控制模式选择，0为位置式，1为增量式，3为不启用模式切换
// pid位置模式相关变量
extern int lastError_left;  // 左轮上次误差(rpm)即目标速度为300而当前速度为0则初始误差为300
extern int lastError_right; // 右轮上次误差(rpm)即目标速度为300而当前速度为0则初始误差为300
// pid增量模式相关变量
extern int incrLastError_left;  // 左轮上次误差
extern int incrPrevError_left;  // 左轮前次误差
extern int incrLastError_right; // 右轮上次误差
extern int incrPrevError_right; // 右轮前次误差
// 陀螺仪相关变量
int grov_check = 1;      // 陀螺仪检测模式选择，0为开启，1为关闭
float *grov_data = NULL; // 用于存储陀螺仪数据的指针变量
// 串口数据缓冲区
uint8_t usart1_txbuf[64] = {0}; // 用于usart1发送缓冲区，电脑串口
uint8_t usart1_rxbuf[64] = {0}; // 用于usart1接收缓冲区，电脑串口
uint8_t usart2_txbuf[64] = {0}; // 用于usart2发送缓冲区，蓝牙串口
uint8_t usart2_rxbuf[64] = {0}; // 用于usart2接收缓冲区，蓝牙串口
uint8_t usart5_txbuf[64] = {0}; // 用于usart5发送缓冲区，k210串口
uint8_t usart5_rxbuf[64] = {0}; // 用于usart5接收缓冲区，k210串口
// 用于串口命令处理
uint8_t *command_point = NULL;           // 用于指向命令字符串中的参数位置
uint16_t rxlen = 0;                      // 实际接收到的数据长度
uint8_t ctrl_flag = 0;                   // 串口控制标志位 - 是否开启串口接收控制（以及复用于初始化后的模式选择循环状态标志）
int8_t beep_flag = 0;                    // 蜂鸣器标志位 - 控制蜂鸣器响的时间
int left_speed = 0;                      // 左轮占空比
int right_speed = 0;                     // 右轮占空比
int goalSpeed_left_temp = DefaultSpeed;  // 左轮逐渐递减减速，防止过快降速导致电机齿轮冲击损坏
int goalSpeed_right_temp = DefaultSpeed; // 右轮逐渐递减减速，防止过快降速导致电机齿轮冲击损坏
// 选择小车工作模式相关变量
// Car_Mode enum_mode = 0; // 小车工作模式枚举体
uint8_t mode_id = 0;         // 选择模式的id
int mode_select_encoder = 0; // 选择模式的旋钮编码器值
/* 私有变量结束 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* 私有函数声明部分 - 在此处添加函数声明 */

/* 私有函数声明结束 */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* 私有函数实现部分 - 在此处实现自定义函数 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart->Instance == USART1) {
        usart1_rxbuf[Size] = '\0'; // 确保字符串以null结尾
        sprintf((char *)usart1_txbuf, "收到[%d]字节数据：%s\r\n", Size, usart1_rxbuf);
        HAL_UART_Transmit(&huart1, usart1_txbuf, strlen((char *)usart1_txbuf), HAL_MAX_DELAY);
        if (ctrl_flag) {
            if (strstr((const char *)usart1_rxbuf, "LED_ON") != NULL) {
                LED_ON;
                printf("成功执行命令[LED_ON] - LED点亮\r\n");
            }
            if (strstr((const char *)usart1_rxbuf, "LED_OFF") != NULL) {
                LED_OFF;
                printf("成功执行命令[LED_OFF] - LED熄灭\r\n");
            }
            if (strstr((const char *)usart1_rxbuf, "BEEP_ON") != NULL) {
                BEEP_ON;
                printf("成功执行命令[BEEP_ON] - 蜂鸣器开启\r\n");
            }
            if (strstr((const char *)usart1_rxbuf, "BEEP_OFF") != NULL) {
                BEEP_OFF;
                printf("成功执行命令[BEEP_OFF] - 蜂鸣器关闭\r\n");
            }
            // 使用sscanf函数只从开始处解析获取数值，需要使用指针参数从字符串中找到参数位置，然后才能执行命令解析
            if ((command_point = (uint8_t *)strstr((const char *)usart1_rxbuf, "left=")) != NULL) {
                sscanf((const char *)command_point, "left=%d", &left_speed);
                motor_control(left_speed, right_speed);
                printf("成功执行命令[left] - 左速度：%d\r\n", left_speed);
            }
            if ((command_point = (uint8_t *)strstr((const char *)usart1_rxbuf, "right=")) != NULL) {
                sscanf((const char *)command_point, "right=%d", &right_speed);
                motor_control(left_speed, right_speed);
                printf("成功执行命令[right] - 右速度：%d\r\n", right_speed);
            }
            if ((command_point = (uint8_t *)strstr((const char *)usart1_rxbuf, "speed=")) != NULL) {
                sscanf((const char *)command_point, "speed=%d,%d", &goalSpeed_left_temp, &goalSpeed_right_temp);
                printf("成功执行命令[speed] - 左速度：%d，右速度%d\r\n", goalSpeed_left_temp, goalSpeed_right_temp);
            }
            if ((command_point = (uint8_t *)strstr((const char *)usart1_rxbuf, "pid mode=")) != NULL) {
                sscanf((const char *)command_point, "pid mode=%d", &pid_mode);
                printf("成功执行命令[pid mode] - pid模式：%s\r\n", pid_mode == 0 ? "位置式" : "增量式");
                // 切换模式时重置相关变量
                // 位置式
                if (pid_mode == 0) {
                    lastError_left = goalSpeed_left - curSpeed_left;    // 左轮上次误差(rpm)，由于首次目标速度为300且当前速度为0，所以初始误差为300
                    lastError_right = goalSpeed_right - curSpeed_right; // 右轮上次误差(rpm)，由于首次目标速度为300且当前速度为0，所以初始误差为300
                }
                // 增量式
                if (pid_mode == 1) {
                    incrLastError_left = goalSpeed_left - curSpeed_left;    // 左轮上次误差
                    incrPrevError_left = goalSpeed_left - curSpeed_left;    // 左轮前次误差
                    incrLastError_right = goalSpeed_right - curSpeed_right; // 右轮上次误差
                    incrPrevError_right = goalSpeed_right - curSpeed_right; // 右轮前次误差
                }
            }
            if ((command_point = (uint8_t *)strstr((const char *)usart1_rxbuf, "grov check=")) != NULL) {
                sscanf((const char *)command_point, "grov check=%d", &grov_check);
                printf("成功执行命令[pid mode] - 陀螺仪检测：%s\r\n", grov_check == 0 ? "关闭" : "开启");
            }
        }
    }
    if (huart->Instance == USART2) {
        usart2_rxbuf[Size] = '\0'; // 确保字符串以null结尾
        printf("[USART2]收到[%d]字节数据：%s\r\n", Size, usart2_rxbuf);
        if (strstr((const char *)usart2_rxbuf, "person") != NULL) {
            BEEP_ON;
            beep_flag = 100; // 设置蜂鸣器持续时间(ms)
            printf("有人！有人！\r\n");
        }
    }
    if (huart->Instance == UART5) { // 确认是UART5中断
        // 使用usart1回显数据
        usart5_rxbuf[Size] = '\0'; // 确保字符串以null结尾
        printf("[USART5]收到[%d]字节数据：%s\r\n", Size, usart5_rxbuf);
        // deal_bluetooth(rx5_temp); // 处理蓝牙数据
    }
    // 重新使能接收中断（HAL库接收一次后会自动关闭中断，需手动重新使能）
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rxbuf, sizeof(usart1_rxbuf));
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, usart2_rxbuf, sizeof(usart2_rxbuf));
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf));
}

static int tim2_cnt = 0; // 定时器计数器变量
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    // 判断是否是目标定时器（避免多个定时器中断冲突）
    if (htim->Instance == TIM2) {
        tim2_cnt++;
        if (beep_flag >= 0) {
            beep_flag -= 10;
        } else {
            BEEP_OFF;
        }
        // 每10ms执行一次PID控制
        if (pid_mode == 0) {
            // 左轮降速缓冲
            if (goalSpeed_left - goalSpeed_left_temp > 5) {
                goalSpeed_left -= 5;
            } else {
                goalSpeed_left = goalSpeed_left_temp;
            }
            // 右轮降速缓冲
            if (goalSpeed_right - goalSpeed_right_temp > 5) {
                goalSpeed_right -= 5;
            } else {
                goalSpeed_right = goalSpeed_right_temp;
            }
            position_control_speed(10);
        } else if (pid_mode == 1) {
            // 左轮降速缓冲
            if (goalSpeed_left - goalSpeed_left_temp > 5) {
                goalSpeed_left -= 5;
            } else {
                goalSpeed_left = goalSpeed_left_temp;
            }
            // 右轮降速缓冲
            if (goalSpeed_right - goalSpeed_right_temp > 5) {
                goalSpeed_right -= 5;
            } else {
                goalSpeed_right = goalSpeed_right_temp;
            }
            increment_control_speed(10);
        }
        if (0) {
            printf("01\r\n");
            // 检查是否发生中断事件  Check if any interruption events have occurred
            Get_Angle(GET_Angle_Way); // 更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果  //Updating the posture once every 5ms, a higher sampling frequency can
                                      // improve the effectiveness of Kalman filtering and complementary filtering
            Encoder_Left = Read_Encoder(0);   // 读取左轮编码器的值，前进为正，后退为负   //Read the value of the left wheel encoder, forward is positive, backward is negative
            Encoder_Right = -Read_Encoder(1); // 读取右轮编码器的值，前进为正，后退为负   //Read the value of the right wheel encoder, forward is positive, backward is negative
            Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right); // 获取速度 Obtain speed

            Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);   // 平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负   //Balance PID control gyro balance angular
                                                                     // velocity polarity: forward tilt is positive, backward tilt is negative
            Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right); // 速度环PID控制	记住，速度反馈是正反馈   //Speed loop PID control. Remember, speed feedback is positive
                                                                     // feedback
            // Velocity_Pwm = 0;
            Motor_Left = Balance_Pwm + Velocity_Pwm + Turn_Pwm;  // 计算左轮电机最终PWM Calculate the final PWM of the left wheel motor
            Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm; // 计算右轮电机最终PWM Calculate the final PWM of the right wheel motor
                                                                 // PWM值正数使小车前进，负数使小车后退 Interrupt callback function

            // 滤掉死区
            Motor_Left = PWM_Ignore(Motor_Left);
            Motor_Right = PWM_Ignore(Motor_Right);

            // PWM限幅 PWM limiting
            Motor_Left = PWM_Limit(Motor_Left, 2600, -2600); // 25khz->2592
            Motor_Right = PWM_Limit(Motor_Right, 2600, -2600);

            Set_Pwm(Motor_Left, Motor_Right); // 赋值给PWM寄存器 	Assign to PWM register
        }
    }
}
int cnt = 0;
// EXTI12
// static u8 exti_print_cnt; // 打印计数器变量，用于外部中断回调函数中，防止频繁打印数据
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

    //
    //		//只有正常模式下检测小车的拿去和放下(姿态检测) Only in normal mode can the detection of the taking and lowering of the car be carried out (posture detection)
    //		if(mode_id == Normal)
    //		{
    //			if(Pick_Up(Acceleration_Z,Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被拿起 Check if the car has been picked up
    //				Stop_Flag=1;	                           					//如果被拿起就关闭电机 If picked up, turn off the motor
    //			if(Put_Down(Angle_Balance,Encoder_Left,Encoder_Right))//检查是否小车被放下 Check if the car has been lowered
    //				Stop_Flag=0;	                           					//如果被放下就启动电机 If it is put down, start the motor
    //		}

    //		if(Turn_Off(Angle_Balance,battery)==0)     					//如果不存在异常 		If there are no abnormalities
    //			motor_control(Motor_Left,Motor_Right);         					//赋值给PWM寄存器 	Assign to PWM register
    if (GPIO_Pin == MPU6050_Int_Pin) {
        // 检查是否发生中断事件  Check if any interruption events have occurred
        Get_Angle(GET_Angle_Way); // 更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果  //Updating the posture once every 5ms, a higher sampling frequency can
                                  // improve the effectiveness of Kalman filtering and complementary filtering
        Encoder_Left = Read_Encoder(0);   // 读取左轮编码器的值，前进为正，后退为负   //Read the value of the left wheel encoder, forward is positive, backward is negative
        Encoder_Right = -Read_Encoder(1); // 读取右轮编码器的值，前进为正，后退为负   //Read the value of the right wheel encoder, forward is positive, backward is negative
        Get_Velocity_Form_Encoder(Encoder_Left, Encoder_Right);  // 获取速度 Obtain speed
        Balance_Pwm = Balance_PD(Angle_Balance, Gyro_Balance);   // 平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负   //Balance PID control gyro balance angular velocity
        Velocity_Pwm = Velocity_PI(Encoder_Left, Encoder_Right); // 速度环PID控制	记住，速度反馈是正反馈   //Speed loop PID control. Remember, speed feedback is positive
                                                                 // feedback
        Turn_Pwm = Turn_PD(Gyro_Turn);
        //			if(cnt++%100==0)
        //				printf("Balance_Pwm=%.d,Velocity_Pwm=%.d\r\n",Balance_Pwm,Velocity_Pwm);
        Motor_Left = Balance_Pwm + Velocity_Pwm + Turn_Pwm;  // 计算左轮电机最终PWM Calculate the final PWM of the left wheel motor
        Motor_Right = Balance_Pwm + Velocity_Pwm - Turn_Pwm; // 计算右轮电机最终PWM Calculate the final PWM of the right wheel motor
                                                             // PWM值正数使小车前进，负数使小车后退 Interrupt callback function

        // 滤掉死区
        Motor_Left = PWM_Ignore(Motor_Left);
        Motor_Right = PWM_Ignore(Motor_Right);

        // PWM限幅 PWM limiting
        Motor_Left = PWM_Limit(Motor_Left, 2600, -2600); // 25khz->2592
        Motor_Right = PWM_Limit(Motor_Right, 2600, -2600);

        Set_Pwm(Motor_Left, Motor_Right); // 赋值给PWM寄存器 	Assign to PWM register
    }
}

/* 私有函数实现结束 */
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

    /* USER CODE BEGIN 1 */
    /* 系统初始化前处理 - 在此处添加HAL初始化之前的代码 */
    // uint32_t adc_val = 0;
    // uint8_t adc_txbuf[64] = {0};
    // double V_bat = 0.0;
    // short left_val = 0;
    // short right_val = 0;

    /* 系统初始化前结束 */
    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */
    /* 时钟初始化前处理 - 在此处添加时钟初始化前的代码 */

    /* 时钟初始化前结束 */
    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */
    /* 系统初始化前处理 - 在此处添加系统初始化前的代码 */

    /* 系统初始化前结束 */
    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_ADC1_Init();
    MX_TIM8_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_UART5_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    /* 程序初始化部分 - 在此处添加进入循环前的代码 */
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
    delay_init();
    /* OLED初始化 */
    printf("OLED Initing...\r\n");
    OLED_I2C_Init();
    OLED_Draw_Line("OLED Initing...", 1, true, true);

    /* 定时器初始化 */
    printf("TIM Initing...\r\n");

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 启动TIM3编码器接口
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动TIM4编码器接口

    /* 初始化模式选择 */
    printf("Mode Selecting...\r\n");
    while (!ctrl_flag) {
        // 获取左右轮编码器的值，有变化时更新模式选择
        mode_select_encoder = encoder_left_getval() + encoder_right_getval();
        if (mode_select_encoder > 1 && (mode_id < Mode_Max - 1)) {
            mode_id++;
            printf("mode=%d\r\n", mode_id);
        } else if (mode_select_encoder < -1 && (mode_id > 0)) {
            mode_id--;
            printf("mode=%d\r\n", mode_id);
        }
        if (KEY_PRESS) {
            // 按键处理
            delay_ms(20);
            if (KEY_PRESS) {
                ctrl_flag = !ctrl_flag;
            }
        }
        OLED_Draw_Line("Select Mode>>>Key1", 1, false, true);
        OLED_Draw_Line(CarModeToString(mode_id), 2, true, true);
    }
    OLED_Draw_Line("Current Mode>>>   ", 1, false, true);
    select_mode(mode_id); // 选择模式，参照enabled_mode_arr数组

    OLED_Draw_Line("System Initing...", 3, false, true);
    printf("System Initing...\r\n");

    /* 串口中断初始化 */
    printf("USART Initing...\r\n");
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, usart1_rxbuf, sizeof(usart1_rxbuf)); // 电脑串口
                                                                              // HAL_UARTEx_ReceiveToIdle_IT(&huart2, usart2_rxbuf, sizeof(usart2_rxbuf)); // K210串口
    HAL_UARTEx_ReceiveToIdle_IT(&huart5, usart5_rxbuf, sizeof(usart5_rxbuf)); // 蓝牙串口
    /* 电机初始化 */
    printf("MOTOR Initing...");
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // 启动TIM8的PWM CH1
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // 启动TIM8的PWM CH2
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // 启动TIM8的PWM CH3
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // 启动TIM8的PWM CH4

    /* 串口初始化 */
    HAL_TIM_Base_Start_IT(&htim2);

    /* 电池检测初始化 */
    Battery_init();

    /* 陀螺仪初始化 */
    printf("Grov Initing...\r\n");

    MPU6050_initialize(); // 陀螺仪初始化 Gyroscope range initialization
    DMP_Init();           // DMP初始化 DMP initialization
    MPU6050_EXTI_Init();  // 平衡服务  Balanced Services

    OLED_Draw_Line("System Init Ready!", 3, false, true);
    printf("System Init Ready!\r\n");

    OLED_Draw_Line("Main While>>>", 4, false, true);
    printf("Main While>>>\r\n");

    /* 程序初始化结束 */
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /* 无限循环部分 - 在此处添加需要循环执行的代码 */
    while (1) {
        // grov_data = Read_DMP();
        //  陀螺仪数据输出

        //        if (tim2_cnt > 100) {
        //            if (grov_check == 1) {
        //                printf("roll=%.1f, pitch=%.1f, yaw=%.1f\n", grov_data[0], grov_data[1], grov_data[2]);
        //            }
        //            tim2_cnt = 0;
        //            printf("curSpeed_left:%d,curSpeed_right:%d\r\n", curSpeed_left, curSpeed_right);
        //						battery = Get_Battery_Volotage();
        //						printf("current battery level:%.2f\r\n",battery);
        //        }

        if (KEY_PRESS) {
            // 按键处理
            delay_ms(20);
            if (KEY_PRESS) {
                ctrl_flag = !ctrl_flag;
                if (ctrl_flag) {
                    printf("串口控制开启\r\n");
                    printf("可用命令：\r\n"
                           "-[LED_ON]：打开LED\r\n"
                           "-[LED_OFF]：关闭LED\r\n"
                           "-[BEEP_ON]：打开蜂鸣器\r\n"
                           "-[BEEP_OFF]：关闭蜂鸣器\r\n"
                           "-[speed=左轮速度,右轮速度]：控制左轮右轮速度，支持正反\r\n"
                           "-[pid mode=0/1]:0为位置式，1为增量式\r\n"
                           "-[grov check=0/1]:0为关闭陀螺仪数据，1为显示陀螺仪数据\r\n ");
                } else {
                    printf("串口控制关闭\r\n");
                }
                while (KEY_PRESS)
                    ; // 等待按键释放
            }
        }
    }
    /* 无限循环结束 */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 无限循环部分 - 在此处添加循环执行的代码 */

    /* 无限循环结束 */
    /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/* 用户自定义函数部分 - 在此处添加用户自定义函数 */

/* 用户自定义函数结束 */
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* 错误处理调试部分 - 在此处添加错误处理的代码 */
    __disable_irq();
    while (1) {
    }
    /* 错误处理调试结束 */
    /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
    /* USER CODE BEGIN 6 */
    /* 断言失败处理函数 - 在此处添加断言失败的处理代码 */
    printf("断言失败: 文件 %s 第 %d 行\r\n", file, line);

    /* 断言失败处理结束 */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
