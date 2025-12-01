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

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
