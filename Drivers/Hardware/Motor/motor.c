#include "motor.h"
#include "tim.h" 
#include "gpio.h" 
#include "pid.h"


PID_Incremental_t pid_motor_a;
PID_Incremental_t pid_motor_b;
PID_Incremental_t pid_motor_c;
PID_Incremental_t pid_motor_d;

PID_Incremental_TypeDef pid_m_a;
PID_Incremental_TypeDef pid_m_b;
PID_Incremental_TypeDef pid_m_c;
PID_Incremental_TypeDef pid_m_d;

// 定义陀螺仪PID结构体
PID_Positional_t pid_gyro;

void All_PID_Init(void)
{
    // 原代码参数: Velocity_KP=1.5, Velocity_KI=10 (代码里除了100, 相当于0.1)
    // 输出限幅 7199
    // PID_Inc_Init(&pid_motor_a, 1.5f, 0.1f, 7199.0f);
    // PID_Inc_Init(&pid_motor_b, 1.5f, 0.1f, 7199.0f);
    // PID_Inc_Init(&pid_motor_c, 1.5f, 0.1f, 7199.0f);
    // PID_Inc_Init(&pid_motor_d, 1.5f, 0.1f, 7199.0f);

    PID_Incremental_Init(&pid_m_a, 1.99f, 1.98f, 0.94f, 7199.0f, -7199.0f);
    PID_Incremental_Init(&pid_m_b, 1.33f, 2.00f, 0.98f, 7199.0f, -7199.0f);
    PID_Incremental_Init(&pid_m_c, 1.43f, 1.75f, 4.04f, 7199.0f, -7199.0f);
    PID_Incremental_Init(&pid_m_d, 1.25f, 0.9f, 0.57f, 7199.0f, -7199.0f);

    PID_Pos_Init(&pid_gyro, 0.0001f, 0.000001f, 0.0001f, 7199.0f);
}



void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

void Servo_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void Servo_Set_PWM(uint16_t pwm_val_1, uint16_t pwm_val_4)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_val_1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm_val_4);
}

void Motor_Set_Speed(int motor_a, int motor_b, int motor_c, int motor_d)
{

    if(motor_a < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, -motor_a);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 7199 - motor_a);
    }

    if(motor_b < 0) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, -motor_b);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 7199 - motor_b);
    }

    if(motor_c < 0) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, -motor_c);
    } else {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 7199 - motor_c);
    }

    if(motor_d < 0) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, -motor_d);
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 7199 - motor_d);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
}