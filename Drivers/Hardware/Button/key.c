#include "key.h"

//HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) // 读取模式按键
//HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) // 读取急停按键

uint8_t Key_Read_Stop(void)
{
    return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
}

//单击和双击检测
uint8_t Key_Scan_Click_Double(uint8_t timeout_ticks)
{
    static uint8_t flag_key = 0;
    static uint8_t count_key = 0;
    static uint8_t double_key = 0;
    static uint16_t count_single = 0;
    static uint16_t Forever_count = 0;

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) // 按键按下
    {
        Forever_count++; 
    }
    else
    {
        Forever_count = 0;
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET && flag_key == 0)
    {
        flag_key = 1; // 标记按键已按下
    }

    if (count_key == 0)
    {
        if (flag_key == 1)
        {
            double_key++;
            count_key = 1; // 锁定，防止在按下期间重复增加 double_key
        }
        
        if (double_key == 2) // 检测到第二次按下
        {
            double_key = 0;
            count_single = 0;
            return 2; // 返回双击
        }
    }

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) // 按键松开
    {
        flag_key = 0;
        count_key = 0;
    }

    if (double_key == 1) // 只有一次按下，正在等待第二次
    {
        count_single++;
        // 超时判定：如果在规定时间内没有第二次按下，且第一次按下时间没有过长
        if (count_single > timeout_ticks && Forever_count < timeout_ticks)
        {
            double_key = 0;
            count_single = 0;
            return 1; // 返回单击
        }
    }
    return 0; // 无动作
}

//长按检测
uint8_t Key_Scan_Long_Press(void)
{
    static uint16_t Long_Press_count = 0;
    static uint8_t Long_Press_State = 0;

    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_SET) // 按下
    {
        if (Long_Press_State == 0) 
            Long_Press_count++;
    }
    else // 松开
    {
        Long_Press_count = 0;
        Long_Press_State = 0; // 复位状态，允许下次检测
    }

    if (Long_Press_count > 200) // 200 * 10ms = 2秒
    {
        Long_Press_State = 1; // 标记已触发，防止重复触发
        Long_Press_count = 0;
        return 1; // 触发长按
    }

    return 0;
}