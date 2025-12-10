#ifndef __USART_H
#define __USART_H

#include "main.h"
#include "stdio.h"

// 接收缓冲区大小
#define RX_BUFFER_SIZE 128
#define TX_BUF_SIZE 64

extern uint8_t tx_buffer[TX_BUF_SIZE]; 

void USART_Init_Logic(void);        // 初始化逻辑
void Send_Speed(float ch1, float ch2, float ch3, float ch4); // DMA发送波形
void Process_Command(void);         // 处理指令
void Uart3_Parse();
void Uart3_Receive();
void USER_UART_IRQHandler_User(UART_HandleTypeDef *huart); // 用户中断入口

#endif