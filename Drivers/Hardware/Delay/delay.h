#ifndef __DELAY_H__
#define __DELAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

// 定义一个全局变量保存每微秒的Tick数，避免每次计算
extern uint32_t SystemCoreClock_MHz; 

/**
 * @brief  Initializes DWT_Cycle_Count
 */
uint32_t DWT_Delay_Init(void);

/**
 * @brief  Microsecond delay using DWT
 * @note   This is a blocking delay. Do not use for long periods in RTOS.
 * Use vTaskDelay() for milliseconds.
 */
__STATIC_INLINE void delay_us(volatile uint32_t microseconds)
{
  uint32_t clk_cycle_start = DWT->CYCCNT;
  
  // 优化：直接使用预先计算好的频率变量，去除除法和函数调用
  microseconds *= SystemCoreClock_MHz;

  /* Delay till end */
  while ((DWT->CYCCNT - clk_cycle_start) < microseconds);
}

#ifdef __cplusplus
}
#endif

#endif /* __DELAY_H__ */