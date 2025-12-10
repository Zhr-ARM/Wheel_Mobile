#include "Encoder.h"

/**
  * @brief  启动所有编码器计数
  */
void Encoder_Init(void)
{
    // 使用 HAL 库启动编码器接口
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

/**
  * @brief  读取编码器计数值，并清零计数器 (用于测量速度)
  * @param  htim: 定时器句柄指针，如 &htim2
  * @retval 速度值 (带符号，单位：脉冲数/周期)
  */
int Get_Encoder_Value(TIM_HandleTypeDef *htim)
{
    int speed_value = 0;
    
    // 1. 读取当前计数值
    // 使用 (short) 强制转换将 uint32_t 的 CNT 转换为有符号 16位整数
    // 这样 65535 就会变成 -1，处理了反转溢出问题
    speed_value = (short)(__HAL_TIM_GET_COUNTER(htim));
    
    // 2. 清零计数器，重新开始计数
    __HAL_TIM_SET_COUNTER(htim, 0);
    
    return speed_value;
}

float Velocity_line_Cal(int encoder)
{
  return ((float)encoder / 54000.0f) * 0.3142f * 100.0f;
}