#include "pid.h"
#include <math.h>

// ======================= 增量式 PID 实现 =======================

void PID_Inc_Init(PID_Incremental_t *pid, float kp, float ki, float limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Output_Limit = limit;
    pid->Error = 0;
    pid->Last_Error = 0;
    pid->Output = 0;
}

// 对应原代码: velocity += Velocity_KP*(bias-Last_bias) + bias*Velocity_KI/100;
float PID_Inc_Calc(PID_Incremental_t *pid, float target, float current)
{
    pid->Error = target - current;
    
    // 增量计算公式
    // 原代码 Ki 除以了 100，我们在初始化传入 Ki 时自己注意除以 100，或者在这里除
    // 这里假设传入的参数已经是调整好的
    float increment = (pid->Kp * (pid->Error - pid->Last_Error)) + 
                      (pid->Ki * pid->Error);

    pid->Output += increment;

    // 更新历史误差
    pid->Last_Error = pid->Error;

    // 输出限幅
    if (pid->Output > pid->Output_Limit)  pid->Output = pid->Output_Limit;
    if (pid->Output < -pid->Output_Limit) pid->Output = -pid->Output_Limit;

    return pid->Output;
}

void PID_Inc_Reset(PID_Incremental_t *pid)
{
    pid->Error = 0;
    pid->Last_Error = 0;
    pid->Output = 0;
}

// ======================= 位置式 PID 实现 =======================

void PID_Pos_Init(PID_Positional_t *pid, float kp, float ki, float kd, float limit)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->Output_Limit = limit;
    pid->Integral_Limit = 5000; // 积分限幅，根据需要调整
    pid->Sum_Error = 0;
    pid->Last_Error = 0;
}

// 对应原代码: V_Z_Control
float PID_Pos_Calc(PID_Positional_t *pid, float target, float current)
{
    pid->Error = target - current;
    
    // 积分累计
    pid->Sum_Error += pid->Error;
    
    // 积分限幅 (原代码 error+=Bias; if(error>10000)...)
    if(pid->Sum_Error > pid->Integral_Limit) pid->Sum_Error = pid->Integral_Limit;
    if(pid->Sum_Error < -pid->Integral_Limit) pid->Sum_Error = -pid->Integral_Limit;
    
    // 计算输出
    // 原代码除以 1000，建议在设置参数时就把 kp, ki, kd 设小，公式里不要除，保持通用性
    float output = (pid->Kp * pid->Error) + 
                   (pid->Ki * pid->Sum_Error) + 
                   (pid->Kd * (pid->Error - pid->Last_Error));
    
    pid->Last_Error = pid->Error;

    // 输出限幅
    if (output > pid->Output_Limit)  output = pid->Output_Limit;
    if (output < -pid->Output_Limit) output = -pid->Output_Limit;

    return output;
}

//--------------------------------------------------------------------------

// 位置式PID初始化
void PID_Positional_Init(PID_Positional_TypeDef *pid, float kp, float ki, float kd, float integral_max, float integral_min, float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->integral_max = integral_max;
    pid->integral_min = integral_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

// 增量式PID初始化
void PID_Incremental_Init(PID_Incremental_TypeDef *pid, float kp, float ki, float kd, float output_max, float output_min)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->error = 0;
    pid->prev_error = 0;
    pid->prev_prev_error = 0;
    pid->output = 0;
    pid->output_max = output_max;
    pid->output_min = output_min;
}

// 位置式PID计算
float PID_Positional_Calc(PID_Positional_TypeDef *pid, float setpoint, float measured)
{
    float error = setpoint - measured;
    if(fabsf(error) < 1.0f)
    {
        error = 0; // 小于1的误差视为0
        pid->integral = 0; // 积分清零
    } 
    pid->integral += error;
    // 积分限幅
    if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if(pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    float derivative = error - pid->prev_error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if(output > pid->output_max) output = pid->output_max;
    if(output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;
}

// 增量式PID计算
float PID_Incremental_Calc(PID_Incremental_TypeDef *pid, float setpoint, float measured)
{
    if(fabsf(setpoint) < 1.0f && fabsf(measured) < 30.0f)
    {
        pid->error = 0;
        pid->prev_error = 0;
        pid->prev_prev_error = 0;
        pid->output = 0;
        return 0;
    }
    pid->error = setpoint - measured;
    float delta_output = pid->kp * (pid->error - pid->prev_error)
                       + pid->ki *  pid->error
                       + pid->kd * (pid->error - 2 * pid->prev_error + pid->prev_prev_error);

    pid->output += delta_output;

    // 输出限幅
    if(pid->output > pid->output_max) pid->output = pid->output_max;
    if(pid->output < pid->output_min) pid->output = pid->output_min;

    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = pid->error;

    return pid->output;
}

float Cascade_PID_Control(PID_Positional_TypeDef *pos_pid,
                         PID_Incremental_TypeDef *vel_pid,
                         float pos_target,
                         float pos_feedback,
                         float vel_feedback)
{
    // 1. 位置环：根据目标位置和当前位置，计算目标速度
    float vel_target = PID_Positional_Calc(pos_pid, pos_target, pos_feedback);

    // 2. 速度环：根据目标速度和当前速度，计算最终输出
    float output = PID_Incremental_Calc(vel_pid, vel_target, vel_feedback);

    return output;
}


// 角度环位置式PID计算
float PID_Angle_Positional_Calc(PID_Positional_TypeDef *pid, float set_angle, float measured_angle)
{
    float error = set_angle - measured_angle;
    
    // 角度环误差处理（防止跨0跳变，假设-180~180度）
    if(error > 180.0f) error -= 360.0f;
    if(error < -180.0f) error += 360.0f;
    if((error<2)&&(error>-2)) error = 0; // 小于2度误差视为0
    
    pid->integral += error;
    // 积分限幅
    if(pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    if(pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    float derivative = error - pid->prev_error;
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 输出限幅
    if(output > pid->output_max) output = pid->output_max;
    if(output < pid->output_min) output = pid->output_min;

    pid->prev_error = error;
    return output;
}
// 角度环增量式PID计算
float PID_Angle_Incremental_Calc_Switch(
    PID_Incremental_TypeDef *pid_coarse,
    PID_Incremental_TypeDef *pid_fine,
    float set_angle,
    float measured_angle,
    float fine_threshold)
{
    float error = set_angle - measured_angle;
    if(error > 180.0f) error -= 360.0f;
    if(error < -180.0f) error += 360.0f;

    // 选择结构体
    PID_Incremental_TypeDef *pid_use = (fabsf(error) < fine_threshold) ? pid_fine : pid_coarse;

    float delta_output = pid_use->kp * (error - pid_use->prev_error)
                       + pid_use->ki * error
                       + pid_use->kd * (error - 2 * pid_use->prev_error + pid_use->prev_prev_error);

    pid_use->output += delta_output;

    if(pid_use->output > pid_use->output_max) pid_use->output = pid_use->output_max;
    if(pid_use->output < pid_use->output_min) pid_use->output = pid_use->output_min;

    pid_use->prev_prev_error = pid_use->prev_error;
    pid_use->prev_error = error;

    return pid_use->output;
}