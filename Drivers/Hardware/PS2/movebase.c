#include "movebase.h"

/*
 * 输出轮子速度
 * @param vel_x: 线速度X分量
 * @param vel_y: 线速度Y分量
 * @param omega: 角速度
 */
//*****************      m/s          m/s         rad/s
void Output_Wheel(float vel_x, float vel_y, float omega, wheel_t *output_wheel)
{
	if (vel_x == 0 && vel_y == 0 && omega == 0)
	{
		output_wheel->leftFront.vel = 0;
		output_wheel->rightFront.vel = 0;
		output_wheel->leftRear.vel = 0;
		output_wheel->rightRear.vel = 0;
		return;
	}

	// 计算各个轮子速度方向
	output_wheel->leftFront = CalcWheel(vel_x, vel_y, omega, LEFT_FRONT_VERTICAL_ANG, output_wheel->angle);
	output_wheel->rightFront = CalcWheel(vel_x, vel_y, omega, RIGHT_FRONT_VERTICAL_ANG, output_wheel->angle);
	output_wheel->leftRear = CalcWheel(vel_x, vel_y, omega, LEFT_REAR_VERTICAL_ANG, output_wheel->angle);
	output_wheel->rightRear = CalcWheel(vel_x, vel_y, omega, RIGHT_REAR_VERTICAL_ANG, output_wheel->angle);

	static float leftFrontAng = 0.0f, rightFrontAng = 0.0f, leftRearAng = 0.0f, rightRearAng = 0.0f;
	// 将定位系统坐标系下角度转换为机器人坐标系下角度 direction-=GetAngle()
	Transform2RobotCoodinate(output_wheel);
	// 将机器人坐标系下角度转换为和电机一致 direction = 90.0f - direction
	Transform2WheelCoodinate(output_wheel);
	// 判断是否需要将轮速反向
	JudgeVelDirection(&output_wheel->leftFront, leftFrontAng);
	JudgeVelDirection(&output_wheel->rightFront, rightFrontAng);
	JudgeVelDirection(&output_wheel->leftRear, leftRearAng);
	JudgeVelDirection(&output_wheel->rightRear, rightRearAng);
	// 保证旋转为劣弧
	leftFrontAng = TurnInferiorArc(output_wheel->leftFront.direction, leftFrontAng);
	rightFrontAng = TurnInferiorArc(output_wheel->rightFront.direction, rightFrontAng);
	leftRearAng = TurnInferiorArc(output_wheel->leftRear.direction, leftRearAng);
	rightRearAng = TurnInferiorArc(output_wheel->rightRear.direction, rightRearAng);

	output_wheel->leftFront.direction = ANGLE2RAD(output_wheel->leftFront.direction);
	output_wheel->rightFront.direction = ANGLE2RAD(output_wheel->rightFront.direction);
	output_wheel->leftRear.direction = ANGLE2RAD(output_wheel->leftRear.direction);
	output_wheel->rightRear.direction = ANGLE2RAD(output_wheel->rightRear.direction);
}

/*
 * 计算各个轮子速度方向
 * @param vel_x: 线速度X分量(m/s)
 * @param vel_y: 线速度Y分量(m/s)
 * @param omega: 角速度(rad/s)
 * @param angleN: 轮子垂直角度(度)
 * @param postureAngle: 机器人姿态角度(度)
 */
// ******************       m/s          m/s         rad/s           度             度
wheelVel_t CalcWheel(float vel_x, float vel_y, float omega, float angleN, float postureAngle)
{
	wheelVel_t sumVel = {0.0f};
	float velX, velY = 0.0f;		  // 平移速度的X，Y分量
	float velN, velNDirection = 0.0f; // 旋转的线速度
	float sumVelX, sumVelY = 0.0f;	  // 合成速度的X，Y分量

	// 计算平移速度的X，Y分量
	velX = vel_x;
	velY = vel_y;
	// 计算旋转的线速度
	velN = omega * MOVEBASE_RADIUS;

	velNDirection = angleN + postureAngle;
	AngleLimit(&velNDirection);

	// 计算和速度大小和方向
	sumVelX = velX + velN * cosf(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * sinf(ANGLE2RAD(velNDirection));

	sumVel.vel = sqrt(sumVelX * sumVelX + sumVelY * sumVelY);

	// 计算合成速度方向时 未将0向量单独处理
	sumVel.direction = RAD2ANGLE(atan2f(sumVelY, sumVelX));

	return sumVel;
}

/**
 * @brief  角度限幅，将角度限制在-180°到180°
 * @note
 * @param  angle:要限制的值
 * @retval
 */
void AngleLimit(float *angle)
{
	static unsigned char recursiveTimes = 0;

	recursiveTimes++;

	if (recursiveTimes < 100)
	{
		if (*angle > 180.0f)
		{
			*angle -= 360.0f;
			AngleLimit(angle);
		}
		else if (*angle < -180.0f)
		{
			*angle += 360.0f;
			AngleLimit(angle);
		}
	}

	recursiveTimes--;
}
// void AngleLimit(float *angle)
// {
// 	while (*angle > 180.0f)
// 	{
// 		*angle -= 360.0f;
// 	}
// 	while (*angle < -180.0f)
// 	{
// 		*angle += 360.0f;
// 	}
// }

/**
 * @brief  将目标角度和实际角度转换到一个360度周期中
 * @note
 * @param  targetAngle:目标角度
 * @param  actualAngle:当前角度
 * @retval 转换后的目标角度
 */
float TurnInferiorArc(float targetAngle, float actualAngle)
{
	if (targetAngle - actualAngle > 180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if (targetAngle - actualAngle < -180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}
}

/**
 * @brief  判断轮子是否需要反转
 * @note
 * @param  targetVel:目标速度大小和方向
 * @param  actualAngle:当前轮子正方向角度
 * @retval
 */
void JudgeVelDirection(wheelVel_t *targetVel, float actualAngle)
{
	int n = 0;
	float angleErr = 0.0f;

	// 将目标角度和当前实际角度转换到一个360度周期中
	n = (int)(actualAngle / 180.0f) - (int)(actualAngle / 360.0f);

	targetVel->direction = n * 360.0f + targetVel->direction;

	// 计算目标角度和实际角度的误差
	angleErr = targetVel->direction - actualAngle;

	// 将误差限制在-180度到180度
	AngleLimit(&angleErr);

	// 如果角度误差大于90度则将速度反向并将目标角度加180度
	if (fabs(angleErr) > 90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;

		// 保证处理后的目标角度和当前实际角度在一个周期中
		if (targetVel->direction > (n * 360.0f + 180.0f))
		{
			targetVel->direction -= 360.0f;
		}
		else if (targetVel->direction < (n * 360.0f - 180.0f))
		{
			targetVel->direction += 360.0f;
		}
	}
}

/**
 * @brief  将机器人坐标系下角度转换为轮子坐标系下角度
 * @note
 * @param  wheelVel
 * @retval
 */
void Transform2WheelCoodinate(wheel_t *wheelVel)
{
	// 将机器人坐标系下轮子朝向转换为轮子坐标系下角度
	wheelVel->leftFront.direction = wheelVel->leftFront.direction - 90.0f;
	wheelVel->rightFront.direction = wheelVel->rightFront.direction - 90.0f;
	wheelVel->leftRear.direction = wheelVel->leftRear.direction - 90.0f;
	wheelVel->rightRear.direction = wheelVel->rightRear.direction - 90.0f;

	// 将角度限制在-180°到180°
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);
}

/**
 * @brief  将定位系统坐标系下角度转换为机器人坐标系下角度
 * @note
 * @param  wheelVel
 * @retval
 */
void Transform2RobotCoodinate(wheel_t *wheelVel)
{
	// 将定位系统坐标系下角度转换为机器人坐标系下角度
	wheelVel->leftFront.direction -= wheelVel->angle;
	wheelVel->rightFront.direction -= wheelVel->angle;
	wheelVel->leftRear.direction -= wheelVel->angle;
	wheelVel->rightRear.direction -= wheelVel->angle;

	// 将角度限制在180度到-180度范围内
	AngleLimit(&wheelVel->leftFront.direction);
	AngleLimit(&wheelVel->rightFront.direction);
	AngleLimit(&wheelVel->leftRear.direction);
	AngleLimit(&wheelVel->rightRear.direction);
}
