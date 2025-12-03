/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#include "math.h"
// 修改为旧代码中的 ID
uint16_t CAN_ID1 = 0x121; 
uint16_t CAN_ID2 = 0x122;
uint16_t CAN_ID3 = 0x123;
uint16_t CAN_ID4 = 0x124;

// 定义 PI 常量（如果未定义）
#ifndef PI
#define PI 3.14159265358979323846
#endif

uint8_t FLAG_CAN = 0;
uint8_t txbuf[8];
uint8_t rxbuf[8]; // 临时接收缓存
uint8_t Rxbuf_1[8], Rxbuf_2[8], Rxbuf_3[8], Rxbuf_4[8]; // 各个ID的数据缓存

// 声明过滤器配置函数
static void CAN_Filter_Config(void);
/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // 2. 在这里配置过滤器并启动 CAN
  CAN_Filter_Config(); // 配置过滤器

  // 启动 CAN 模块
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  // 开启接收 FIFO0 消息挂号中断
  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_CAN1_2();

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static void CAN_Filter_Config(void)
{
  CAN_FilterTypeDef sFilterConfig;

  sFilterConfig.FilterBank = 0;                       // 过滤器组0
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;   // 屏蔽位模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;  // 32位宽
  sFilterConfig.FilterIdHigh = 0x0000;                // ID高16位
  sFilterConfig.FilterIdLow = 0x0000;                 // ID低16位
  sFilterConfig.FilterMaskIdHigh = 0x0000;            // 掩码高16位 (0表示不关心，接收所有)
  sFilterConfig.FilterMaskIdLow = 0x0000;             // 掩码低16位
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;  // 关联到 FIFO0
  sFilterConfig.FilterActivation = ENABLE;            // 激活过滤器
  sFilterConfig.SlaveStartFilterBank = 14;            // 单CAN一般设为14

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

// 发送一条消息
// 返回值: 0~2 (邮箱号), 0xFF (无空邮箱)
u8 CAN1_Tx_Msg(u32 id, u8 ide, u8 rtr, u8 len, u8 *dat)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailboxMask;
    u8 mbox_index = 0xFF;

    // 1. 检查是否有空闲邮箱
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0) return 0xFF;

    // 2. 填充发送头
    TxHeader.StdId = id;        // 标准ID
    TxHeader.ExtId = id;        // 扩展ID (如果ide=1)
    TxHeader.IDE = (ide == 0) ? CAN_ID_STD : CAN_ID_EXT;
    TxHeader.RTR = (rtr == 0) ? CAN_RTR_DATA : CAN_RTR_REMOTE;
    TxHeader.DLC = len;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 3. 发送数据
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, dat, &TxMailboxMask) != HAL_OK)
    {
        return 0xFF;
    }

    // 4. 将 HAL 返回的掩码转换为邮箱索引 (0, 1, 2) 以兼容你的旧代码
    if (TxMailboxMask == CAN_TX_MAILBOX0) mbox_index = 0;
    else if (TxMailboxMask == CAN_TX_MAILBOX1) mbox_index = 1;
    else if (TxMailboxMask == CAN_TX_MAILBOX2) mbox_index = 2;

    return mbox_index;
}

// 获得发送状态
// mbox: 邮箱编号 (0, 1, 2)
// 返回值: 0x07 (发送成功/空闲), 0x00 (正在发送/挂起), 0x05 (失败/邮箱号错误)
u8 CAN1_Tx_Staus(u8 mbox)
{
    // HAL库查询指定邮箱是否挂起
    uint32_t mailbox_mask;

    if (mbox == 0) mailbox_mask = CAN_TX_MAILBOX0;
    else if (mbox == 1) mailbox_mask = CAN_TX_MAILBOX1;
    else if (mbox == 2) mailbox_mask = CAN_TX_MAILBOX2;
    else return 0x05; // 邮箱号错误

    // 检查该邮箱是否还在 Pending (挂起/正在发送)
    if (HAL_CAN_IsTxMessagePending(&hcan, mailbox_mask))
    {
        return 0x00; // 正在发送 (挂起)
    }
    else
    {
        return 0x07; // 发送成功 (HAL库里如果不Pending了，通常就是发出去了)
    }
}

// 查询接收 FIFO 中的报文个数
// fifox: 0 或 1
u8 CAN1_Msg_Pend(u8 fifox)
{
    if (fifox == 0) return HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0);
    else if (fifox == 1) return HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO1);
    else return 0;
}

// 接收数据 (底层函数)
// 这里的实现逻辑是：从 FIFO 读取数据并填充到你的指针里
void CAN1_Rx_Msg(u8 fifox, u32 *id, u8 *ide, u8 *rtr, u8 *len, u8 *dat)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint32_t fifo;

    if (fifox == 0) fifo = CAN_RX_FIFO0;
    else fifo = CAN_RX_FIFO1;

    // 使用 HAL 库读取
    if (HAL_CAN_GetRxMessage(&hcan, fifo, &RxHeader, dat) == HAL_OK)
    {
        *ide = (RxHeader.IDE == CAN_ID_STD) ? 0 : 1;
        if (*ide == 0) *id = RxHeader.StdId;
        else *id = RxHeader.ExtId;

        *rtr = (RxHeader.RTR == CAN_RTR_DATA) ? 0 : 1;
        *len = RxHeader.DLC;
    }
}

// 发送一组固定数据 (ID 0x601)
u8 CAN1_Send_Msg(u8* msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(0X601, 0, 0, len, msg);
    
    // 等待发送结束 (兼容原来的等待逻辑)
    while ((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++;
    
    if (i >= 0XFFF) return 1; // 发送失败/超时
    return 0;                 // 发送成功
}

// 发送测试数据 (ID 0x701)
u8 CAN1_Send_MsgTEST(u8* msg, u8 len)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(0X701, 0, 0, len, msg);
    while ((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++;
    if (i >= 0XFFF) return 1;
    return 0;
}

// 指定 ID 发送数据
u8 CAN1_Send_Num(u32 id, u8* msg)
{
    u8 mbox;
    u16 i = 0;
    mbox = CAN1_Tx_Msg(id, 0, 0, 8, msg);
    while ((CAN1_Tx_Staus(mbox) != 0X07) && (i < 0XFFF)) i++;
    if (i >= 0XFFF) return 1;
    return 0;
}

// 简单的发送封装
void CAN1_SEND(int id, u8* temp)
{
    CAN1_Send_Num((u32)id, temp);
}

// 查询式接收数据
u8 CAN1_Receive_Msg(u8 *buf)
{
    u32 id;
    u8 ide, rtr, len;
    
    if (CAN1_Msg_Pend(0) == 0) return 0; // 没有数据
    
    CAN1_Rx_Msg(0, &id, &ide, &rtr, &len, buf);
    
    // 这里保留你原来的校验逻辑 (只收 0x12 ?) 
    // 注意：原来的代码写的是 id!=0x12，这会过滤掉除了0x12以外的所有数据。
    // 如果这是你想要的逻辑，请保留。
    if (id != 0x12 || ide != 0 || rtr != 0) len = 0; 
    
    return len;
}

// ---------------------------------------------------------
// 中断回调函数 (替代原来的 USB_LP_CAN1_RX0_IRQHandler)
// ---------------------------------------------------------
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t i;

    // 只有当通过中断方式接收时，这里才会被触发
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rxbuf) == HAL_OK)
    {
        // 你的数据处理逻辑
        if (RxHeader.StdId == CAN_ID1)
        {
            for (i = 0; i < 8; i++) { Rxbuf_1[i] = rxbuf[i]; rxbuf[i] = 0; }
            FLAG_CAN = 1;
        }
        else if (RxHeader.StdId == CAN_ID2)
        {
            for (i = 0; i < 8; i++) { Rxbuf_2[i] = rxbuf[i]; rxbuf[i] = 0; }
            FLAG_CAN = 1;
        }
        else if (RxHeader.StdId == CAN_ID3)
        {
            for (i = 0; i < 8; i++) { Rxbuf_3[i] = rxbuf[i]; rxbuf[i] = 0; }
            FLAG_CAN = 1;
        }
        else if (RxHeader.StdId == CAN_ID4)
        {
            for (i = 0; i < 8; i++) { Rxbuf_4[i] = rxbuf[i]; rxbuf[i] = 0; }
            FLAG_CAN = 1;
        }
    }
}

void CAN1_Receive_data(void) //具体数据接收在can中断服务函数里，这里做数据处理
{
        if(FLAG_CAN==1)
					{
				    if(Rxbuf_1[0]==0XAA&&Rxbuf_1[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_1[i];//和校验确认
					      if(Rxbuf_1[6]==sum){fault|=1;	
                  Current_angle_A =  -((short)(Rxbuf_1[2]|(Rxbuf_1[3]<<8)) + (float)((short)(Rxbuf_1[4]|(Rxbuf_1[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_1[0]=0XFF;						
				      }

				    if(Rxbuf_2[0]==0XAA&&Rxbuf_2[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_2[i];//和校验确认
					      if(Rxbuf_2[6]==sum){fault|=2;
                  Current_angle_B =  -((short)(Rxbuf_2[2]|(Rxbuf_2[3]<<8)) + (float)((short)(Rxbuf_2[4]|(Rxbuf_2[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_2[0]=0XFF;				
				      }

				    if(Rxbuf_3[0]==0XAA&&Rxbuf_3[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_3[i];//和校验确认
					      if(Rxbuf_3[6]==sum){fault|=4;
                  Current_angle_C =  -((short)(Rxbuf_3[2]|(Rxbuf_3[3]<<8)) + (float)((short)(Rxbuf_3[4]|(Rxbuf_3[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_3[0]=0XFF;				
				      }

				    if(Rxbuf_4[0]==0XAA&&Rxbuf_4[7]==0XAF)
							{//确认帧头和帧尾					
					      u8 sum=0;
					      for(u8 i=1;i<6;i++)sum+=Rxbuf_4[i];//和校验确认
					      if(Rxbuf_4[6]==sum){fault|=8;
                  Current_angle_D =  -((short)(Rxbuf_4[2]|(Rxbuf_4[3]<<8)) + (float)((short)(Rxbuf_4[4]|(Rxbuf_4[5]<<8)))/100.f);//动作指令3
					      }				
				 	      sum=0;Rxbuf_4[0]=0XFF;				
				      }
					  
					  FLAG_CAN_ON=0;
				    FLAG_CAN=0;
				  }	
         Angle_current_A = 	Current_angle_A + Angle_error_A;
         Angle_current_B = 	Current_angle_B + Angle_error_B;
         Angle_current_C = 	Current_angle_C + Angle_error_C;
         Angle_current_D = 	Current_angle_D + Angle_error_D;					
}

/**
 * @brief 发送电机控制数据 (移植自原 main.c)
 */
void CAN1_SEND_data(uint8_t EN_A, uint8_t EN_B, uint8_t EN_C, uint8_t EN_D, float angle_A, float angle_B, float angle_C, float angle_D) 
{
    static uint8_t sum = 0;
    static uint8_t Time_step = 0;
    short data_A, data_B, data_C, data_D;
    
    // 计算角度的小数部分
    data_A = (short)(angle_A * 100) % 100;
    data_B = (short)(angle_B * 100) % 100;    
    data_C = (short)(angle_C * 100) % 100;    
    data_D = (short)(angle_D * 100) % 100;    
    
    if(FLAG_CAN_ON < 30)
    {        
         Time_step++;
         
         // --- 发送 A 轮 ---
         if(Time_step == 1)
         {
            sum = 0;
            txbuf[0] = 0XAA; // 帧头
            txbuf[1] = EN_A; 
            txbuf[2] = (short)angle_A >> 0;    
            txbuf[3] = (short)angle_A >> 8;        
            txbuf[4] = (short)data_A >> 0;      
            txbuf[5] = (short)data_A >> 8;    
            for(uint8_t i = 1; i < 6; i++) sum += txbuf[i];    
            txbuf[6] = sum; // 校验位
            txbuf[7] = 0xAF; // 帧尾                        
            
            CAN1_Send_Num(CAN_ID1, txbuf); // 发送
         }
         
         // --- 发送 B 轮 ---
         if(Time_step == 2)
         {
            sum = 0;
            txbuf[0] = 0XAA;
            txbuf[1] = EN_B; 
            txbuf[2] = (short)angle_B >> 0;    
            txbuf[3] = (short)angle_B >> 8;        
            txbuf[4] = (short)data_B >> 0;      
            txbuf[5] = (short)data_B >> 8;    
            for(uint8_t i = 1; i < 6; i++) sum += txbuf[i];    
            txbuf[6] = sum;
            txbuf[7] = 0xAF;                        
            
            CAN1_Send_Num(CAN_ID2, txbuf);
         } 
         
         // --- 发送 C 轮 ---
         if(Time_step == 3)
         {
            sum = 0;
            txbuf[0] = 0XAA;
            txbuf[1] = EN_C; 
            txbuf[2] = (short)angle_C >> 0;    
            txbuf[3] = (short)angle_C >> 8;        
            txbuf[4] = (short)data_C >> 0;      
            txbuf[5] = (short)data_C >> 8;    
            for(uint8_t i = 1; i < 6; i++) sum += txbuf[i];    
            txbuf[6] = sum;
            txbuf[7] = 0xAF;                        
            
            CAN1_Send_Num(CAN_ID3, txbuf);
         } 
         
         // --- 发送 D 轮 ---
         if(Time_step == 4)
         {
            sum = 0;
            txbuf[0] = 0XAA;
            txbuf[1] = EN_D; 
            txbuf[2] = (short)angle_D >> 0;    
            txbuf[3] = (short)angle_D >> 8;        
            txbuf[4] = (short)data_D >> 0;      
            txbuf[5] = (short)data_D >> 8;    
            for(uint8_t i = 1; i < 6; i++) sum += txbuf[i];    
            txbuf[6] = sum;
            txbuf[7] = 0xAF;                        
            
            CAN1_Send_Num(CAN_ID4, txbuf);
            
            Time_step = 0;    
         }                    
    }  
}

/* USER CODE BEGIN 4 */

/* USER CODE END 1 */
