#ifndef __MOVEBASE_H
#define __MOVEBASE_H

#include <math.h>
#include <assert.h>
#include <stdio.h>
#define MAX_SPEED 200 // 电机最大转速
// 角度制转化为弧度制
#define ANGLE2RAD(x) (x / 180.0f * PI)
// 弧度制转换为角度制
#define RAD2ANGLE(x) (x / PI * 180.0f)
// 底盘旋转半径
#define MOVEBASE_RADIUS (282.8427f)
//#define MOVEBASE_RADIUS (0.2828427f)
// 左前轮与中心连线切线方向
#define LEFT_FRONT_VERTICAL_ANG (-35.0f) //-135.0f
// 右前轮与中心连线切线方向
#define RIGHT_FRONT_VERTICAL_ANG (35.0f) // 135.0f
// 左后轮与中心连线切线方向
#define LEFT_REAR_VERTICAL_ANG (-145.0f) //-45.0f
// 右后轮与中心连线切线方向
#define RIGHT_REAR_VERTICAL_ANG (145.0f) // 45.0f

#define PI 3.1415926f
typedef struct
{
	// 轮子速度大小
	float vel;
	// 轮子速度方向
	float direction;
} wheelVel_t;

typedef struct
{
	// 左前轮
	wheelVel_t leftFront;
	// 右前轮
	wheelVel_t rightFront;
	// 左后轮
	wheelVel_t leftRear;
	// 右后轮
	wheelVel_t rightRear;
	// 机器人姿态角度,没有则设置为90
	float angle;
} wheel_t;

void Output_Wheel(float vel_x, float vel_y, float omega, wheel_t *output_wheel); // 输出轮子速度

wheelVel_t CalcWheel(float vel, float direction, float omega, float angleN, float postureAngle); // 计算轮子速度
void AngleLimit(float *angle);																	 // 角度限幅
void JudgeVelDirection(wheelVel_t *targetVel, float actualAngle);								 // 判断轮子速度方向
void Transform2WheelCoodinate(wheel_t *wheelVel);												 // 将机器人坐标系下的角度转换为轮子坐标系下的角度
void Transform2RobotCoodinate(wheel_t *wheelVel);												 // 将轮子坐标系下的角度转换为机器人坐标系下的角度
float TurnInferiorArc(float targetAngle, float actualAngle);									 // 保证旋转为劣弧

#endif
