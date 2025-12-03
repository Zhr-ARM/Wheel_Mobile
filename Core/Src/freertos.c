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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId oledHandle;
osThreadId ps2Handle;
osThreadId angle_motorHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
wheel_t wheel={{0,0},{0,0},{0,0},{0,0},90.0};
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Oled_show(void const * argument);
void PS2_recv(void const * argument);
void Angle_motor_control(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

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
void MX_FREERTOS_Init(void) {
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
  osThreadDef(oled, Oled_show, osPriorityAboveNormal, 0, 512);
  oledHandle = osThreadCreate(osThread(oled), NULL);

  /* definition and creation of ps2 */
  osThreadDef(ps2, PS2_recv, osPriorityAboveNormal, 0, 256);
  ps2Handle = osThreadCreate(osThread(ps2), NULL);

  /* definition and creation of angle_motor */
  osThreadDef(angle_motor, Angle_motor_control, osPriorityRealtime, 0, 512);
  angle_motorHandle = osThreadCreate(osThread(angle_motor), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  // osDelay(200);         // 新增：上电后先等待200ms，让外设电源稳定
  // MPU6050_initialize(); // MPU6050初始化
  // osDelay(10);          // 延时
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
void Oled_show(void const * argument)
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
void PS2_recv(void const * argument)
{
  /* USER CODE BEGIN PS2_recv */
  PS2_SetInit(); // PS2手柄初始化
  /* Infinite loop */
  for (;;)
  {
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
void Angle_motor_control(void const * argument)
{
  /* USER CODE BEGIN Angle_motor_control */
  /* Infinite loop */
  for (;;)
  {
    CAN1_Receive_data();
    if (Start_Flag == 1 && Flag_STOP == 0)
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
    Output_Wheel((int)(127-PS2_RY), (int)(PS2_RX-128), (int)(PS2_LY-127), &wheel);
    // CAN1_SEND_data(CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D,
    //                 RAD2ANGLE(wheel.rightFront.direction), RAD2ANGLE(wheel.leftFront.direction),
    //                 RAD2ANGLE(wheel.leftRear.direction), RAD2ANGLE(wheel.rightRear.direction));
    CAN1_SEND_data(CAN_EN_A, CAN_EN_B, CAN_EN_C, CAN_EN_D,
                    0, 0,
                    0, 0);
    printf("a:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Angle_current_A,Angle_current_B,Angle_current_C,Angle_current_D,RAD2ANGLE(wheel.rightFront.direction),RAD2ANGLE(wheel.leftFront.direction),RAD2ANGLE(wheel.leftRear.direction),RAD2ANGLE(wheel.rightRear.direction));
    // osDelay(1000);
    //HAL_UART_Transmit(&huart1, (uint8_t *)"hello\n", 6, 10);
    osDelay(50);
  }
  /* USER CODE END Angle_motor_control */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

