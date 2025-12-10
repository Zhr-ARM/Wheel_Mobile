#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

void Motor_Init(void);

void Servo_Init(void);

//-7199 µ½ +7199
void Motor_Set_Speed(int motor_a, int motor_b, int motor_c, int motor_d);

void Servo_Set_PWM(uint16_t pwm_val_1, uint16_t pwm_val_4);

void All_PID_Init(void);
#endif