#include "delay.h"

uint32_t SystemCoreClock_MHz = 0; // 保存每微秒的Tick数

uint32_t DWT_Delay_Init(void) {
  /* 1. 获取系统时钟并转换为 MHz (Cycles per us) */
  /* 如果你的系统时钟是 168MHz，这里就是 168 */
  SystemCoreClock_MHz = HAL_RCC_GetHCLKFreq() / 1000000;
  
  /* 防错：如果时钟获取失败，默认给一个值，防止死锁 */
  if (SystemCoreClock_MHz == 0) SystemCoreClock_MHz = 1;

  /* Disable TRC */
  CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
  /* Enable TRC */
  CoreDebug->DEMCR |=  CoreDebug_DEMCR_TRCENA_Msk;

  /* Disable clock cycle counter */
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  __ASM volatile ("NOP");
  __ASM volatile ("NOP");
  __ASM volatile ("NOP");

  if(DWT->CYCCNT) {
    return 0; /* Success */
  } else {
    return 1; /* Error */
  }
}