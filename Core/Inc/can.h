/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN_Init(void);

/* USER CODE BEGIN Prototypes */
extern uint16_t CAN_ID1, CAN_ID2, CAN_ID3, CAN_ID4;
extern uint8_t FLAG_CAN;
extern uint8_t txbuf[8];
extern uint8_t rxbuf[8];
extern uint8_t Rxbuf_1[8], Rxbuf_2[8], Rxbuf_3[8], Rxbuf_4[8];

// 2. 声明功能函数
u8 CAN1_Tx_Msg(u32 id, u8 ide, u8 rtr, u8 len, u8 *dat); // 底层发送
u8 CAN1_Tx_Staus(u8 mbox);                             // 检查状态
u8 CAN1_Msg_Pend(u8 fifox);                            // 检查有无数据
void CAN1_Rx_Msg(u8 fifox, u32 *id, u8 *ide, u8 *rtr, u8 *len, u8 *dat); // 底层接收

u8 CAN1_Send_Msg(u8* msg, u8 len);       // 发送特定ID 0x601
u8 CAN1_Send_MsgTEST(u8* msg, u8 len);   // 发送测试ID 0x701
u8 CAN1_Send_Num(u32 id, u8* msg);       // 发送任意ID
void CAN1_SEND(int id, u8* temp);        // 简单封装

u8 CAN1_Receive_Msg(u8 *buf);            // 查询方式接收

void CAN1_Receive_data(void);

void CAN1_SEND_data(uint8_t EN_A, uint8_t EN_B, uint8_t EN_C, uint8_t EN_D, float angle_A, float angle_B, float angle_C, float angle_D);
void Car_Swerve_Control(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

