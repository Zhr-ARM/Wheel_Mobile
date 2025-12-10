/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "mpu6050.h"
#include "ps2.h"
#include "math.h"
#include "can.h"
#include "movebase.h"
#include "usart.h"
#include "pid.h"
#include "motor.h"
#include "Encoder.h"
#include "Uart.h"
#include "key.h"
#include "ROS_Uart.h"
#include "adc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
//-------------------------//
// ROS的输入
float input_vx_ROS = 0;
float input_vy_ROS = 0;
float input_omega_ROS = 0;
//-------------------------//
float set_a = 0;
float set_b = 0;
float set_c = 0;
float set_d = 0;

float input_vx = 0;
float input_vy = 0;
float input_omega = 0;

static uint32_t last_toggle_tick = 0;
static uint8_t key_processed = 0;

volatile uint8_t Stop_Key_State = 0;
volatile uint8_t Last_Key_State = 1;
volatile uint8_t Stop_Emergency = 0;
volatile uint8_t Select_Control_Mode = 0; // 0: PS2, 1: ROS

float Velocity_A = 0;
float Velocity_B = 0;
float Velocity_C = 0;
float Velocity_D = 0;

float Last_pwm_a = 0.0f;
float Last_pwm_b = 0.0f;
float Last_pwm_c = 0.0f;
float Last_pwm_d = 0.0f;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
float Encoder_A_Speed, Encoder_B_Speed, Encoder_C_Speed, Encoder_D_Speed;

extern PID_Incremental_t pid_motor_a;
extern PID_Incremental_t pid_motor_b;
extern PID_Incremental_t pid_motor_c;
extern PID_Incremental_t pid_motor_d;
extern uint8_t Start_Flag_My;

float Target_Speed_A = 500;
float Target_Speed_B = -1000;
float Target_Speed_C = -500;
float Target_Speed_D = 500;

extern PID_Incremental_TypeDef pid_m_a;
extern PID_Incremental_TypeDef pid_m_b;
extern PID_Incremental_TypeDef pid_m_c;
extern PID_Incremental_TypeDef pid_m_d;
int txlen = 0;

// 2. PID 计算
float pwm_a = 0, pwm_b = 0, pwm_c = 0, pwm_d = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId oledHandle;
osThreadId ps2Handle;
osThreadId angle_motorHandle;
osThreadId EncoderHandle;
osThreadId MotorHandle;
osThreadId JetsonHandle;
osThreadId ADCHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
wheel_t wheel = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, 90.0};

float PWM_Smooth_Limit(float target_pwm, float *last_pwm)
{
  float output_pwm = target_pwm;
  float delta = target_pwm - *last_pwm;
  const float MAX_STEP = 50.0f;

  if (fabs(target_pwm) > fabs(*last_pwm))
  {
    if (delta > MAX_STEP)
    {
      output_pwm = *last_pwm + MAX_STEP;
    }
    else if (delta < -MAX_STEP)
    {
      output_pwm = *last_pwm - MAX_STEP;
    }
  }
  else
  {
    output_pwm = target_pwm;
  }

  *last_pwm = output_pwm;

  if (output_pwm > 7199.0f)
    output_pwm = 7199.0f;
  if (output_pwm < -7199.0f)
    output_pwm = -7199.0f;

  return output_pwm;
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const *argument);
void Oled_show(void const *argument);
void PS2_recv(void const *argument);
void Angle_motor_control(void const *argument);
void Encoder_Four(void const *argument);
void Motor_Set(void const *argument);
void Jetson_Receive(void const *argument);
void Get_Vol(void const *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize);

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of oled */
  osThreadDef(oled, Oled_show, osPriorityBelowNormal, 0, 512);
  oledHandle = osThreadCreate(osThread(oled), NULL);

  /* definition and creation of ps2 */
  osThreadDef(ps2, PS2_recv, osPriorityAboveNormal, 0, 256);
  ps2Handle = osThreadCreate(osThread(ps2), NULL);

  /* definition and creation of angle_motor */
  osThreadDef(angle_motor, Angle_motor_control, osPriorityRealtime, 0, 512);
  angle_motorHandle = osThreadCreate(osThread(angle_motor), NULL);

  /* definition and creation of Encoder */
  osThreadDef(Encoder, Encoder_Four, osPriorityHigh, 0, 256);
  EncoderHandle = osThreadCreate(osThread(Encoder), NULL);

  /* definition and creation of Motor */
  osThreadDef(Motor, Motor_Set, osPriorityRealtime, 0, 128);
  MotorHandle = osThreadCreate(osThread(Motor), NULL);

  /* definition and creation of Jetson */
  osThreadDef(Jetson, Jetson_Receive, osPriorityRealtime, 0, 512);
  JetsonHandle = osThreadCreate(osThread(Jetson), NULL);

  /* definition and creation of ADC */
  osThreadDef(ADC, Get_Vol, osPriorityBelowNormal, 0, 128);
  ADCHandle = osThreadCreate(osThread(ADC), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 * @test  LED blinks every 500 ms
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  PS2_LX = 128;
  PS2_LY = 128;
  PS2_RX = 128;
  PS2_RY = 128;
  // osDelay(200);         // 新增：上电后先等待200ms，让外设电源稳定
  // MPU6050_initialize(); // MPU6050初始化
  // osDelay(100);         // 延时
  // DMP_Init();           // 初始化DMP
  // osDelay(10);          // 延时

  /* Infinite loop */
  for (;;)
  {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Oled_show */
/**
 * @brief Function implementing the oled thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Oled_show */
void Oled_show(void const *argument)
{
  /* USER CODE BEGIN Oled_show */
  OLED_Init(); /* Infinite loop */
  for (;;)
  {
    oled_show();
    osDelay(5);
  }
  /* USER CODE END Oled_show */
}

/* USER CODE BEGIN Header_PS2_recv */
/**
 * @brief Function implementing the ps2 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_PS2_recv */
void PS2_recv(void const *argument)
{
  /* USER CODE BEGIN PS2_recv */
  PS2_SetInit(); // PS2手柄初始化
  /* Infinite loop */
  for (;;)
  {
    Stop_Key_State = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
    PS2_Control();
    osDelay(5);
  }
  /* USER CODE END PS2_recv */
}

/* USER CODE BEGIN Header_Angle_motor_control */
/**
 * @brief Function implementing the angle_motor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Angle_motor_control */
void Angle_motor_control(void const *argument)
{
  /* USER CODE BEGIN Angle_motor_control */

  uint32_t stuck_count = 0;
  uint8_t last_rx = 0, last_ry = 0, last_lx = 0, last_ly = 0;

  /* Infinite loop */
  for (;;)
  {
    CAN1_Receive_data();
    if (Start_Flag_My == 1 && Flag_STOP == 0)
    {
      CAN_EN_A = 1;
      CAN_EN_B = 1;
      CAN_EN_C = 1;
      CAN_EN_D = 1;
    }
    else
    {
      CAN_EN_A = 0;
      CAN_EN_B = 0;
      CAN_EN_C = 0;
      CAN_EN_D = 0;
    }
    // 检测手柄数据是否卡死
    if (PS2_RX == last_rx && PS2_RY == last_ry && PS2_LX == last_lx && PS2_LY == last_ly)
    {
      stuck_count++;
    }
    else
    {
      // 数据有变化，重置计数器和历史值
      stuck_count = 0;
      last_rx = PS2_RX;
      last_ry = PS2_RY;
      last_lx = PS2_LX;
      last_ly = PS2_LY;
    }

    // d断链状态
    //  全0或全255处于非正中状态，且数据长时间没有任何变化

    int is_disconnected = 0;

    if ((PS2_RX == 0 && PS2_RY == 0 && PS2_LX == 0 && PS2_LY == 0) ||
        (PS2_RX == 255 && PS2_RY == 255 && PS2_LX == 255 && PS2_LY == 255))
    {
      is_disconnected = 1;
    }

    if (is_disconnected)
    {
      // 断开后清零变量
      input_vx = 0;
      input_vy = 0;
      input_omega = 0;

      // 定义全局变量，防止连接后起飞
      PS2_LX = 128;
      PS2_LY = 128;
      PS2_RX = 128;
      PS2_RY = 128;
    }
    else
    {
      input_vx = (int)(127 - PS2_RY);
      input_vy = (int)(PS2_RX - 128);

      float raw_omega = (float)(PS2_LY - 127);
      input_omega = raw_omega / 250.0f; // 除以一个较大的数，减小旋转速度

      input_vx_ROS = Velocity_MID_X * 1000.0f; // m/s -> mm/s
      input_vy_ROS = Velocity_MID_Y * 1000.0f; // m/s -> mm/s

      // 角速度保持不变，因为 omega * 282 自动变成了 mm/s
      input_omega_ROS = Velocity_MID_Z;
    }

    // Output_Wheel((int)(127 - PS2_RY), (int)(PS2_RX - 128), (int)(PS2_LY - 127), &wheel);
    // Output_Wheel((int)(127 - PS2_RY), (int)(PS2_RX - 128), (int)(PS2_LY - 127), &wheel);

    if ((PS2_KEY == 1))
    {
      uint32_t current_tick = xTaskGetTickCount();

      if (key_processed == 0 && (current_tick - last_toggle_tick > 200))
      {
        Select_Control_Mode = !Select_Control_Mode;
        last_toggle_tick = current_tick;
        key_processed = 1;
      }
    }
    if (Select_Control_Mode == 0)
    {
      Output_Wheel(input_vx, input_vy, input_omega, &wheel);
    }
    else
    {
      Output_Wheel(input_vx_ROS, input_vy_ROS, input_omega_ROS, &wheel);
    }
    // Output_Wheel(input_vx, input_vy, input_omega, &wheel);
    //  Output_Wheel(input_vx_ROS, input_vy_ROS, input_omega_ROS, &wheel);
    CAN1_SEND_data(CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D,
                   RAD2ANGLE(wheel.rightFront.direction), RAD2ANGLE(wheel.leftFront.direction),
                   RAD2ANGLE(wheel.leftRear.direction), RAD2ANGLE(wheel.rightRear.direction));
    // CAN1_SEND_data(CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D,
    //                 0, 0,
    //                 0, 0);
    printf("a:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n", Angle_current_A, Angle_current_B, Angle_current_C, Angle_current_D, RAD2ANGLE(wheel.rightFront.direction), RAD2ANGLE(wheel.leftFront.direction), RAD2ANGLE(wheel.leftRear.direction), RAD2ANGLE(wheel.rightRear.direction));
    //  osDelay(1000);
    //  HAL_UART_Transmit(&huart1, (uint8_t *)"hello\n", 6, 10);
    osDelay(5);
  }
  /* USER CODE END Angle_motor_control */
}

/* USER CODE BEGIN Header_Encoder_Four */
/**
 * @brief Function implementing the Encoder thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Encoder_Four */
void Encoder_Four(void const *argument)
{
  /* USER CODE BEGIN Encoder_Four */
  // Encoder_Init();
  USART_Init_Logic();
  /* Infinite loop */
  for (;;)
  {

    //Send_Speed(Velocity_A, Velocity_B, Velocity_C, Velocity_D);
    // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Velocity_D, sizeof(Velocity_D));
    osDelay(5);
  }
  /* USER CODE END Encoder_Four */
}

/* USER CODE BEGIN Header_Motor_Set */
/**
 * @brief Function implementing the Motor thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Motor_Set */
void Motor_Set(void const *argument)
{
  /* USER CODE BEGIN Motor_Set */
  Encoder_Init();
  Motor_Init(); // 电机初始化
  All_PID_Init();
  float pwm_factor = 1.719f;

  const float DIR_A = 1.0f;
  const float DIR_B = -1.0f;
  const float DIR_C = -1.0f;
  const float DIR_D = 1.0f;

  /* Infinite loop */
  for (;;)
  {
    // Uart3_Parse();
    if ((PS2_KEY == 4))
    {
      uint32_t current_tick = xTaskGetTickCount();

      if (key_processed == 0 && (current_tick - last_toggle_tick > 200))
      {
        Start_Flag_My = !Start_Flag_My;
        last_toggle_tick = current_tick;
        key_processed = 1;
      }
    }
    else
    {
      key_processed = 0;
    }

    if (Stop_Key_State == 0)
    {
      Stop_Emergency = 1;
      Start_Flag_My = 0;
    }
    else
    {
      Stop_Emergency = 0;
    }
    if (Start_Flag_My == 0 || Flag_STOP == 1 || Stop_Emergency == 1)
    {
      Motor_Set_Speed(0, 0, 0, 0);

      Last_pwm_a = 0;
      Last_pwm_b = 0;
      Last_pwm_c = 0;
      Last_pwm_d = 0;

      osDelay(10);
      continue;
    }

    // Motor_Set_Speed((int)pwm_a, (int)pwm_b, (int)pwm_c, (int)pwm_d);
    // //Motor_Set_Speed(100, 100, 100, 100);

    // set_a = wheel.rightFront.vel * pwm_factor;
    // set_b = wheel.leftFront.vel * pwm_factor;
    // set_c = wheel.leftRear.vel * pwm_factor;
    // set_d = wheel.rightRear.vel * pwm_factor;

    set_a = wheel.rightFront.vel * pwm_factor * DIR_A;
    set_b = wheel.leftFront.vel * pwm_factor * DIR_B;
    set_c = wheel.leftRear.vel * pwm_factor * DIR_C;
    set_d = wheel.rightRear.vel * pwm_factor * DIR_D;

    if (set_a > 7199)
      set_a = 7199;
    else if (set_a < -7199)
      set_a = -7199;
    if (set_b > 7199)
      set_b = 7199;
    else if (set_b < -7199)
      set_b = -7199;
    if (set_c > 7199)
      set_c = 7199;
    else if (set_c < -7199)
      set_c = -7199;
    if (set_d > 7199)
      set_d = 7199;
    else if (set_d < -7199)
      set_d = -7199;

    int raw_enc_a = Get_Encoder_Value(&htim5);
    int raw_enc_b = Get_Encoder_Value(&htim3);
    int raw_enc_c = Get_Encoder_Value(&htim2);
    int raw_enc_d = Get_Encoder_Value(&htim4);

    Encoder_A_Speed = (float)raw_enc_a;
    Encoder_B_Speed = (float)raw_enc_b;
    Encoder_C_Speed = (float)raw_enc_c;
    Encoder_D_Speed = (float)raw_enc_d;

    Velocity_A = Velocity_line_Cal(raw_enc_a);
    Velocity_B = Velocity_line_Cal(raw_enc_b);
    Velocity_C = Velocity_line_Cal(raw_enc_c);
    Velocity_D = Velocity_line_Cal(raw_enc_d);

    float raw_pwm_a = PID_Incremental_Calc(&pid_m_a, set_a, Encoder_A_Speed);
    float raw_pwm_b = PID_Incremental_Calc(&pid_m_b, set_b, Encoder_B_Speed);
    float raw_pwm_c = PID_Incremental_Calc(&pid_m_c, set_c, Encoder_C_Speed);
    float raw_pwm_d = PID_Incremental_Calc(&pid_m_d, set_d, Encoder_D_Speed);

    pwm_a = PWM_Smooth_Limit(raw_pwm_a, &Last_pwm_a);
    pwm_b = PWM_Smooth_Limit(raw_pwm_b, &Last_pwm_b);
    pwm_c = PWM_Smooth_Limit(raw_pwm_c, &Last_pwm_c);
    pwm_d = PWM_Smooth_Limit(raw_pwm_d, &Last_pwm_d);

    // // Motor_Set_Speed((int)set_a, (int)set_b, (int)set_c, (int)set_d);
    // Motor_Set_Speed((int)pwm_a, (int)pwm_b, (int)pwm_c, (int)pwm_d);

    Motor_Set_Speed((int)pwm_a, (int)pwm_b, (int)pwm_c, (int)pwm_d);

    osDelay(10);
  }
  /* USER CODE END Motor_Set */
}

/* USER CODE BEGIN Header_Jetson_Receive */
/**
 * @brief Function implementing the Jetson thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Jetson_Receive */
void Jetson_Receive(void const *argument)
{
  /* USER CODE BEGIN Jetson_Receive */
  ROS_Uart_Init();
  /* Infinite loop */
  for (;;)
  {
    ROS_Uart_Task();
    osDelay(5);
  }
  /* USER CODE END Jetson_Receive */
}

/* USER CODE BEGIN Header_Get_Vol */
/**
 * @brief Function implementing the ADC thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Get_Vol */
void Get_Vol(void const *argument)
{
  /* USER CODE BEGIN Get_Vol */
  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END Get_Vol */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
