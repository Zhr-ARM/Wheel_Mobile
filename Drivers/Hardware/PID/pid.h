#ifndef __PID_H
#define __PID_H

#include "main.h"

// --------------------- 1. 增量式 PID  ---------------------
typedef struct {
    float Kp;
    float Ki;
    
    float Error;        // 当前误差
    float Last_Error;   // 上次误差
    
    float Output;       
    float Output_Limit; 
    
} PID_Incremental_t;

// --------------------- 2. 位置式 PID  ---------------------
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    
    float Error;        
    float Sum_Error;    
    float Last_Error;   
    
    float Output;       
    float Integral_Limit; 
    float Output_Limit;   
    
} PID_Positional_t;


// 初始化增量式PID参数
void PID_Inc_Init(PID_Incremental_t *pid, float kp, float ki, float limit);

// 计算增量式PID (替代原 velocityA_Control)
// target: 目标速度, current: 当前编码器值
float PID_Inc_Calc(PID_Incremental_t *pid, float target, float current);

// 重置PID状态 (停车或重新开始时调用)
void PID_Inc_Reset(PID_Incremental_t *pid);


// 初始化位置式PID参数
void PID_Pos_Init(PID_Positional_t *pid, float kp, float ki, float kd, float limit);

// 计算位置式PID (替代原 V_Z_Control)
// target: 目标角度, current: 当前角度
float PID_Pos_Calc(PID_Positional_t *pid, float target, float current);

//------------------------------------------------------

// 位置式PID结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float integral_max;
    float integral_min;
    float output_max;
    float output_min;
} PID_Positional_TypeDef;

// 增量式PID结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float error;
    float prev_error;
    float prev_prev_error;
    float output;
    float output_max;
    float output_min;
} PID_Incremental_TypeDef;

// 位置式PID初始化
void PID_Positional_Init(PID_Positional_TypeDef *pid, float kp, float ki, float kd, float integral_max, float integral_min, float output_max, float output_min);

// 增量式PID初始化
void PID_Incremental_Init(PID_Incremental_TypeDef *pid, float kp, float ki, float kd, float output_max, float output_min);

// 位置式PID计算
float PID_Positional_Calc(PID_Positional_TypeDef *pid, float setpoint, float measured);

// 增量式PID计算
float PID_Incremental_Calc(PID_Incremental_TypeDef *pid, float setpoint, float measured);

// 串级PID控制函数
float Cascade_PID_Control(PID_Positional_TypeDef *pos_pid,
                         PID_Incremental_TypeDef *vel_pid,
                         float pos_target,
                         float pos_feedback,
                         float vel_feedback);

// 角度环位置式PID
float PID_Angle_Positional_Calc(PID_Positional_TypeDef *pid, float set_angle, float measured_angle);

// 角度环增量式PID
float PID_Angle_Incremental_Calc_Switch(
    PID_Incremental_TypeDef *pid_coarse,
    PID_Incremental_TypeDef *pid_fine,
    float set_angle,
    float measured_angle,
    float fine_threshold);

#endif