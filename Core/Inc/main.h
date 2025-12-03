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
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include <string.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern float Roll,Pitch,Yaw,gyro_Roll,gyro_Pitch,gyro_Yaw ,accel_x,accel_y,accel_z;//XYZ三轴旋转角度、角速度
extern uint8_t PS2_KEY,PS2_LX,PS2_LY,PS2_RX,PS2_RY;
extern uint8_t Start_Flag;                    //启动标志
extern uint8_t  Flag_STOP;               //启动标志
extern uint16_t FLAG_CAN_ON;
extern uint16_t CAN_ID1,CAN_ID2,CAN_ID3,CAN_ID4;
extern uint8_t  CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D, can_ser;
extern float Angle_current_A,Angle_current_B,Angle_current_C,Angle_current_D;
extern float Angle_error_A,Angle_error_B,Angle_error_C,Angle_error_D;
extern float Current_angle_A,Current_angle_B,Current_angle_C,Current_angle_D;
extern uint8_t abnormal,fault;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SDA_Pin GPIO_PIN_13
#define OLED_SDA_GPIO_Port GPIOC
#define OLED_RES_Pin GPIO_PIN_14
#define OLED_RES_GPIO_Port GPIOC
#define OLED_DC_Pin GPIO_PIN_15
#define OLED_DC_GPIO_Port GPIOC
#define IO1_Pin GPIO_PIN_1
#define IO1_GPIO_Port GPIOC
#define IO2_Pin GPIO_PIN_2
#define IO2_GPIO_Port GPIOC
#define IO3_Pin GPIO_PIN_3
#define IO3_GPIO_Port GPIOC
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define CIN2_Pin GPIO_PIN_4
#define CIN2_GPIO_Port GPIOC
#define DI_Pin GPIO_PIN_5
#define DI_GPIO_Port GPIOC
#define IO4_Pin GPIO_PIN_0
#define IO4_GPIO_Port GPIOB
#define key2_Pin GPIO_PIN_1
#define key2_GPIO_Port GPIOB
#define IO5_Pin GPIO_PIN_2
#define IO5_GPIO_Port GPIOB
#define INT_Pin GPIO_PIN_12
#define INT_GPIO_Port GPIOB
#define INT_EXTI_IRQn EXTI15_10_IRQn
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOB
#define EN_Pin GPIO_PIN_14
#define EN_GPIO_Port GPIOB
#define DIN2_Pin GPIO_PIN_15
#define DIN2_GPIO_Port GPIOB
#define key1_Pin GPIO_PIN_12
#define key1_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_12
#define OLED_SCL_GPIO_Port GPIOC
#define DO_Pin GPIO_PIN_2
#define DO_GPIO_Port GPIOD
#define CLK_Pin GPIO_PIN_4
#define CLK_GPIO_Port GPIOB
#define CS_Pin GPIO_PIN_5
#define CS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;  /*!< Read Only */
typedef const int16_t sc16;  /*!< Read Only */
typedef const int8_t sc8;   /*!< Read Only */

typedef __IO int32_t  vs32;
typedef __IO int16_t  vs16;
typedef __IO int8_t   vs8;

typedef __I int32_t vsc32;  /*!< Read Only */
typedef __I int16_t vsc16;  /*!< Read Only */
typedef __I int8_t vsc8;   /*!< Read Only */

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;  /*!< Read Only */
typedef const uint16_t uc16;  /*!< Read Only */
typedef const uint8_t uc8;   /*!< Read Only */

typedef __IO uint32_t  vu32;
typedef __IO uint16_t vu16;
typedef __IO uint8_t  vu8;

typedef __I uint32_t vuc32;  /*!< Read Only */
typedef __I uint16_t vuc16;  /*!< Read Only */
typedef __I uint8_t vuc8;   /*!< Read Only */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
