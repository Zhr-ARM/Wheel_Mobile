#ifndef __ROS_UART_H
#define __ROS_UART_H

#include "main.h"
#include "usart.h"

#define ROS_RX_LEN 64
#define ROS_TX_LEN 512

// ----------------传感器----------------//
extern float Encoder_A_Speed, Encoder_B_Speed, Encoder_C_Speed, Encoder_D_Speed; // 对应 Velocity_line_A 等
extern float Velocity_A, Velocity_B, Velocity_C, Velocity_D;                     // 对应物理速度
extern float Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D;
extern float gyro_Roll, gyro_Pitch, gyro_Yaw;
extern float accel_x, accel_y, accel_z;
extern float Roll, Pitch, Yaw;
extern int Voltage; // 临时定义在 freertos.c
extern uint8_t Start_Flag;

// ----------------Jetson数据----------------//
extern float Angle_MID_A, Angle_MID_B, Angle_MID_C, Angle_MID_D;
extern float Velocity_MID_A, Velocity_MID_B, Velocity_MID_C, Velocity_MID_D;
extern float Velocity_MID_X, Velocity_MID_Y, Velocity_MID_Z;
extern uint8_t Flag_Mode;
extern float Flag_init;

extern volatile uint32_t ROS_Rx_Count;
extern volatile uint32_t ROS_Rx_Valid_Count; 
extern volatile uint32_t ROS_Rx_Error_Count;
// 里程计
extern double Delta_x, Delta_y, Delta_th;

void ROS_Uart_Init(void);
void ROS_Uart_Task(void);                            // 主任务
void ROS_Uart_IRQHandler(UART_HandleTypeDef *huart); // 中断入口

#endif