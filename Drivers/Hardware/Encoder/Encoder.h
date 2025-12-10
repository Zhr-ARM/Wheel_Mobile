#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "tim.h"

//初始化编码器
void Encoder_Init(void);

//读取编码器数值并清零
int Get_Encoder_Value(TIM_HandleTypeDef *htim);

float Velocity_line_Cal(int encoder);

#endif