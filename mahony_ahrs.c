#include "mahony_ahrs.h"
#include <math.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x) * 0.01745329251994329576923690768489f)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x) * 57.295779513082320876798154814105f)
#endif

// 快速平方根倒数算法
static mahony_real_t invSqrt(mahony_real_t x) {
    mahony_real_t halfx = 0.5f * x;
    mahony_real_t y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(mahony_real_t*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void Mahony_Init(Mahony_Handle_t *handle, mahony_real_t Kp, mahony_real_t Ki) {
    handle->q0 = 1.0f;
    handle->q1 = 0.0f;
    handle->q2 = 0.0f;
    handle->q3 = 0.0f;
    
    handle->exInt = 0.0f;
    handle->eyInt = 0.0f;
    handle->ezInt = 0.0f;
    
    handle->Kp = Kp;
    handle->Ki = Ki;
}

void Mahony_SetInitialBias(Mahony_Handle_t *handle, mahony_real_t bx, mahony_real_t by, mahony_real_t bz) {
#if MAHONY_INPUT_IS_DEGREES
    handle->exInt = DEG2RAD(bx); 
    handle->eyInt = DEG2RAD(by);
    handle->ezInt = DEG2RAD(bz);
#else
    handle->exInt = bx;
    handle->eyInt = by;
    handle->ezInt = bz;
#endif
}

void Mahony_Update(Mahony_Handle_t *handle, 
                   mahony_real_t ax, mahony_real_t ay, mahony_real_t az, 
                   mahony_real_t gx, mahony_real_t gy, mahony_real_t gz, 
                   mahony_real_t dt) 
{
    mahony_real_t recipNorm;
    mahony_real_t halfvx, halfvy, halfvz;
    mahony_real_t halfex, halfey, halfez;
    mahony_real_t qa, qb, qc;

#if MAHONY_INPUT_IS_DEGREES
    gx = DEG2RAD(gx);
    gy = DEG2RAD(gy);
    gz = DEG2RAD(gz);
#endif

    // 如果加速度计数据有效
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // 归一化加速度
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // 估计重力方向 (参考系 -> 机体系)
        halfvx = handle->q1 * handle->q3 - handle->q0 * handle->q2;
        halfvy = handle->q0 * handle->q1 + handle->q2 * handle->q3;
        halfvz = handle->q0 * handle->q0 - 0.5f + handle->q3 * handle->q3; 

        // 计算误差 (叉积)
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // 积分反馈 (消除 Bias)
        if(handle->Ki > 0.0f) {
            handle->exInt += handle->Ki * halfex * 2.0f * dt;
            handle->eyInt += handle->Ki * halfey * 2.0f * dt;
            handle->ezInt += handle->Ki * halfez * 2.0f * dt;
            
            gx += handle->exInt;
            gy += handle->eyInt;
            gz += handle->ezInt;
        } else {
            handle->exInt = 0.0f;
            handle->eyInt = 0.0f;
            handle->ezInt = 0.0f;
        }

        // 比例反馈
        gx += handle->Kp * halfex * 2.0f;
        gy += handle->Kp * halfey * 2.0f;
        gz += handle->Kp * halfez * 2.0f;
    }

    // 四元数微分方程求解
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);
    
    qa = handle->q0;
    qb = handle->q1;
    qc = handle->q2;
    
    handle->q0 += (-qb * gx - qc * gy - handle->q3 * gz);
    handle->q1 += (qa * gx + qc * gz - handle->q3 * gy);
    handle->q2 += (qa * gy - qb * gz + handle->q3 * gx);
    handle->q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化四元数
    recipNorm = invSqrt(handle->q0 * handle->q0 + handle->q1 * handle->q1 + handle->q2 * handle->q2 + handle->q3 * handle->q3);
    handle->q0 *= recipNorm;
    handle->q1 *= recipNorm;
    handle->q2 *= recipNorm;
    handle->q3 *= recipNorm;
}

void Mahony_GetEulerAngle(Mahony_Handle_t *handle, mahony_real_t *roll, mahony_real_t *pitch, mahony_real_t *yaw) {
    mahony_real_t q0 = handle->q0;
    mahony_real_t q1 = handle->q1;
    mahony_real_t q2 = handle->q2;
    mahony_real_t q3 = handle->q3;

    // Roll (x-axis rotation)
    mahony_real_t sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    mahony_real_t cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    *roll = atan2f(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    mahony_real_t sinp = 2.0f * (q0 * q2 - q1 * q3);
    if (fabs(sinp) >= 1.0f)
        *pitch = copysignf(MAHONY_PI / 2.0f, sinp);
    else
        *pitch = asinf(sinp);

    // Yaw (z-axis rotation)
    mahony_real_t siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    mahony_real_t cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    *yaw = atan2f(siny_cosp, cosy_cosp);
    
    // 转换为角度
    *roll = RAD2DEG(*roll);
    *pitch = RAD2DEG(*pitch);
    *yaw = RAD2DEG(*yaw);
}
