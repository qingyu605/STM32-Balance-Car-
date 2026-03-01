/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_oled.h"
#include "bsp_oled_i2c.h"
#include "delay.h"
#include "encoder.h"
#include "motor.h"
#include "mpu6050.h"
#include "myenum.h"
#include "oled_fonts.h"
#include "pid.h"
#include "bluetooth.h"
#include "mode.h"
#include "control.h"
#include "battery.h"
#include "KF.h"
#include "filter.h"
// C language header file
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// HAL library STM32 header file
#include "adc.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern float Velocity_Left,Velocity_Right; 								//���ӵ��ٶ� The speed of the wheels
extern uint8_t GET_Angle_Way;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�  Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
extern float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//ƽ����� ƽ�������� ת�������� Balance tilt angle balance gyroscope steering gyroscope
extern int Motor_Left,Motor_Right;                 	  		//���PWM���� Motor PWM variable
extern int Temperature;                                		//�¶ȱ��� Temperature variable
extern float Acceleration_Z;                           		//Z����ٶȼ�  Z-axis accelerometer
extern int Mid_Angle;                          				//��е��ֵ  Mechanical median
extern float Move_X,Move_Z;															//Move_X:ǰ���ٶȣ�Forward speed��  Move_Z��ת���ٶ�(Turning speed)
extern float battery; 																	//��ص���	battery level 
extern uint8_t lower_power_flag; 														//�͵�ѹ��־,��ѹ�ָ���־ Low voltage sign, voltage recovery sign
//extern u32 g_distance; 																	//����������ֵ Ultrasonic distance value
//extern u8 Flag_velocity; 																//�ٶȿ�����ر��� Speed control related variables
//extern enCarState g_newcarstate; 												//С��״̬��־ Car status indicator
extern uint8_t Stop_Flag;																		//ֹͣ��־  Stop sign
//extern  float Car_Target_Velocity,Car_Turn_Amplitude_speed; //ǰ���ٶ� ��ת�ٶ�  Forward speed and rotational speed
extern int Mid_Angle;																		//��е��ֵ  Mechanical median
extern uint8_t mode_id; 																	//ģʽѡ��  Mode selection


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define MPU6050_SDA_Pin GPIO_PIN_10
#define MPU6050_SDA_GPIO_Port GPIOB
#define MPU6050_SCL_Pin GPIO_PIN_11
#define MPU6050_SCL_GPIO_Port GPIOB
#define Test_Pin GPIO_PIN_12
#define Test_GPIO_Port GPIOB
#define KEY_Pin GPIO_PIN_8
#define KEY_GPIO_Port GPIOA
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define BEEP_Pin GPIO_PIN_11
#define BEEP_GPIO_Port GPIOA
#define MPU6050_Int_Pin GPIO_PIN_12
#define MPU6050_Int_GPIO_Port GPIOA
#define MPU6050_Int_EXTI_IRQn EXTI15_10_IRQn
#define USART5_TX_Pin GPIO_PIN_12
#define USART5_TX_GPIO_Port GPIOC
#define USART5_RX_Pin GPIO_PIN_2
#define USART5_RX_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_3
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#ifndef u8
#define u8 uint8_t
#endif

#ifndef u16
#define u16 uint16_t
#endif

#ifndef u32
#define u32 uint32_t
#endif

#define LED_ON HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)
#define LED_OFF HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)
#define LED_TURN HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

#define KEY_PRESS HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET

#define BEEP_ON HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET)
#define BEEP_OFF HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET)
#define BEEP_TURN HAL_GPIO_TogglePin(BEEP_GPIO_Port, BEEP_Pin)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
