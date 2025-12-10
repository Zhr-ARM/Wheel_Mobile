#include "Uart.h" // 包含你的项目总头文件
#include "ROS_Uart.h"
#include "math.h"
#include "string.h"
#include "usart.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;
uint8_t ros_tx_buffer[ROS_TX_LEN];
uint8_t ros_rx_buffer[ROS_RX_LEN];
volatile uint8_t ros_rx_flag = 0;

uint8_t ros_rx_buffer_processing[ROS_RX_LEN]; // 任务的缓存

// 控制目标值
float Angle_MID_A, Angle_MID_B, Angle_MID_C, Angle_MID_D;
float Velocity_MID_A, Velocity_MID_B, Velocity_MID_C, Velocity_MID_D;
float Velocity_MID_X, Velocity_MID_Y, Velocity_MID_Z;
uint8_t Flag_Mode = 0;
float Flag_init = 1.0f;

volatile uint32_t ROS_Rx_Count = 0;       // 接收中断次数
volatile uint32_t ROS_Rx_Valid_Count = 0; // 校验通过次数
volatile uint32_t ROS_Rx_Error_Count = 0; // 校验失败次数

// 里程计
double vx = 0.0, vy = 0.0, vth = 0.0;
double Delta_x = 0.0, Delta_y = 0.0, Delta_th = 0.0;

// 二进制转float
float b2f(uint8_t m0, uint8_t m1, uint8_t m2, uint8_t m3)
{
    float sig = 1.0f;
    if (m0 >= 128.0f)
        sig = -1.0f;
    float jie = (m0 >= 128.0f) ? (m0 - 128.0f) : m0;
    jie = jie * 2.0f;
    if (m1 >= 128.0f)
        jie += 1.0f;
    jie -= 127.0f;
    float tail = 0.0f;
    if (m1 >= 128.0f)
        m1 -= 128.0f;
    tail = m3 + (m2 + m1 * 256.0f) * 256.0f;
    tail = tail / 8388608.0f;
    return sig * pow(2.0, jie) * (1 + tail);
}

void ROS_Uart_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, ros_rx_buffer, ROS_RX_LEN);
}

// ----------------里程计解算--------------//
void Odom_xyz(float vel_a, float vel_b, float vel_c, float vel_d, float ang_a, float ang_b, float ang_c, float ang_d)
{
    const float to_rad = 0.017453f;
    float Power_A_X = vel_a * cos(ang_a * to_rad);
    float Power_A_Y = vel_a * sin(ang_a * to_rad);
    float Power_B_X = vel_b * cos(ang_b * to_rad);
    float Power_B_Y = vel_b * sin(ang_b * to_rad);
    float Power_C_X = vel_c * cos(ang_c * to_rad);
    float Power_C_Y = vel_c * sin(ang_c * to_rad);
    float Power_D_X = vel_d * cos(ang_d * to_rad);
    float Power_D_Y = vel_d * sin(ang_d * to_rad);

    vx = (Power_A_X + Power_B_X + Power_C_X + Power_D_X) / 4.0f;
    vy = (Power_A_Y + Power_B_Y + Power_C_Y + Power_D_Y) / 4.0f;
    vth = gyro_Yaw * 0.001064f;

    double dt = 0.005; // 5ms
    double delta_x = (vx * cos(Delta_th) - vy * sin(Delta_th)) * dt;
    double delta_y = (vx * sin(Delta_th) + vy * cos(Delta_th)) * dt;
    double delta_th = vth * dt;

    Delta_x += delta_x;
    Delta_y += delta_y;
    Delta_th += delta_th;

    if (Flag_init == 0)
    {
        Flag_init = 1;
        Delta_x = 0;
        Delta_y = 0;
        Delta_th = 0;
    }
}

// 发送
void Send_data_ROS_Full(void)
{
    if (huart1.gState != HAL_UART_STATE_READY)
        return;

    int len = sprintf((char *)ros_tx_buffer,
                      "ST:%d,AG:%.1f,%.1f,%.1f,%.1f,VL:%.3f,%.3f,%.3f,%.3f,GY:%.3f,%.3f,%.3f,AC:%.3f,%.3f,%.3f,RPY:%.1f,%.1f,%.1f,Vol:%d,X:%.3f,Y:%.3f,Th:%.3f\r\n",

                      /*<01> Start_Flag */ Start_Flag,
                      /*<02-05> Angle   */ Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D,
                      /*<06-09> Vel     */ Velocity_A, Velocity_B, Velocity_C, Velocity_D, // 假设这是计算好的 m/s
                      /*<10-12> Gyro    */ gyro_Roll, gyro_Pitch, gyro_Yaw,
                      /*<13-15> Accel   */ accel_x, accel_y, accel_z,
                      /*<16-18> RPY     */ Roll, Pitch, Yaw,
                      /*<19> Voltage    */ Voltage, // 临时定义在freeertos.c
                      /*<20-22> Odom    */ (float)Delta_x, (float)Delta_y, (float)Delta_th);

    HAL_UART_Transmit_DMA(&huart1, ros_tx_buffer, len);
}

void ROS_Uart_Task(void)
{
    Odom_xyz(Velocity_A, Velocity_B, Velocity_C, Velocity_D, Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D);
    // Send_data_ROS_Full();

    if (ros_rx_flag)
    {
        uint8_t *pBuf = ros_rx_buffer_processing;

        if (pBuf[0] == 0xAA && pBuf[1] == 0xAA)
        {
            uint8_t len = pBuf[3];

            if (len > (ROS_RX_LEN - 5))
            {
                ROS_Rx_Error_Count++;
                ros_rx_flag = 0;
                return;
            }

            uint8_t checksum = 0;
            uint8_t received_sum = pBuf[len + 4];

            for (uint8_t i = 0; i < (len + 4); i++)
            {
                checksum += pBuf[i];
            }

            if (checksum == received_sum)
            {
                ROS_Rx_Valid_Count++;

                Start_Flag = (uint8_t)b2f(pBuf[4], pBuf[5], pBuf[6], pBuf[7]);
                Flag_Mode = (uint8_t)b2f(pBuf[8], pBuf[9], pBuf[10], pBuf[11]);

                if (Flag_Mode == 0) // 角度控制模式
                {
                    Angle_MID_A = b2f(pBuf[12], pBuf[13], pBuf[14], pBuf[15]);
                    Angle_MID_B = b2f(pBuf[16], pBuf[17], pBuf[18], pBuf[19]);
                    Angle_MID_C = b2f(pBuf[20], pBuf[21], pBuf[22], pBuf[23]);
                    Angle_MID_D = b2f(pBuf[24], pBuf[25], pBuf[26], pBuf[27]);

                    Velocity_MID_A = b2f(ros_rx_buffer[28], ros_rx_buffer[29], ros_rx_buffer[30], ros_rx_buffer[31]);
                    Velocity_MID_B = b2f(pBuf[32], pBuf[33], pBuf[34], pBuf[35]);
                    Velocity_MID_C = b2f(pBuf[36], pBuf[37], pBuf[38], pBuf[39]);
                    Velocity_MID_D = b2f(pBuf[40], pBuf[41], pBuf[42], pBuf[43]);

                    Flag_init = b2f(pBuf[44], pBuf[45], pBuf[46], pBuf[47]);
                }
                else if (Flag_Mode == 1) // 速度控制模式
                {
                    Velocity_MID_X = b2f(pBuf[12], pBuf[13], pBuf[14], pBuf[15]);
                    Velocity_MID_Y = b2f(pBuf[16], pBuf[17], pBuf[18], pBuf[19]);
                    Velocity_MID_Z = b2f(pBuf[20], pBuf[21], pBuf[22], pBuf[23]);

                    Flag_init = b2f(pBuf[44], pBuf[45], pBuf[46], pBuf[47]);
                }
            }
            else
            {
                ROS_Rx_Error_Count++;
            }
        }
        else
        {
            ROS_Rx_Error_Count++;
        }

        ros_rx_flag = 0;
    }
}
void ROS_Uart_IRQHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);

            HAL_UART_DMAStop(huart);

            memcpy(ros_rx_buffer_processing, ros_rx_buffer, ROS_RX_LEN);

            HAL_UART_Receive_DMA(huart, ros_rx_buffer, ROS_RX_LEN);

            ROS_Rx_Count++;
            ros_rx_flag = 1;
        }
    }
}