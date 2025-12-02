#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

#include <math.h>

#define mahony_real_t float

// 1: 输入角速度单位为 度/秒 (deg/s)
// 0: 输入角速度单位为 弧度/秒 (rad/s)
#define MAHONY_INPUT_IS_DEGREES 1

#ifndef MAHONY_PI
#define MAHONY_PI 3.14159265358979323846f
#endif

typedef struct {
    mahony_real_t q0, q1, q2, q3;     // 四元数
    mahony_real_t exInt, eyInt, ezInt;// 积分误差累积
    
    mahony_real_t Kp; // 比例增益: 控制加速度计修正陀螺仪的速度
    mahony_real_t Ki; // 积分增益: 控制陀螺仪零偏(Bias)的消除速度
    
} Mahony_Handle_t;


// 声明句柄
#define MAHONY_DECLARE_HANDLE(name) Mahony_Handle_t name

#define MAHONY_INIT(handle_ptr, kp, ki) Mahony_Init(handle_ptr, kp, ki)

// 设置初始零偏 (可选)
#define MAHONY_SET_BIAS(handle_ptr, bx, by, bz) Mahony_SetInitialBias(handle_ptr, bx, by, bz)

// 更新算法
// dt: 两次调用之间的时间间隔(秒)
#define MAHONY_UPDATE(handle_ptr, ax, ay, az, gx, gy, gz, dt) Mahony_Update(handle_ptr, ax, ay, az, gx, gy, gz, dt)

// 获取欧拉角 (单位: 度)
#define MAHONY_GET_EULER(handle_ptr, roll_ptr, pitch_ptr, yaw_ptr) Mahony_GetEulerAngle(handle_ptr, roll_ptr, pitch_ptr, yaw_ptr)

void Mahony_Init(Mahony_Handle_t *handle, mahony_real_t Kp, mahony_real_t Ki);
void Mahony_SetInitialBias(Mahony_Handle_t *handle, mahony_real_t bx, mahony_real_t by, mahony_real_t bz);
void Mahony_Update(Mahony_Handle_t *handle, 
                   mahony_real_t ax, mahony_real_t ay, mahony_real_t az, 
                   mahony_real_t gx, mahony_real_t gy, mahony_real_t gz, 
                   mahony_real_t dt);
void Mahony_GetEulerAngle(Mahony_Handle_t *handle, mahony_real_t *roll, mahony_real_t *pitch, mahony_real_t *yaw);

#endif // __MAHONY_AHRS_H
