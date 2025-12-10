#ifndef __BSP_KEY_H
#define __BSP_KEY_H

#include "main.h" 

#define KEY_NONE   0
#define KEY_SINGLE 1
#define KEY_DOUBLE 2
#define KEY_LONG   3


uint8_t Key_Read_Stop(void);
uint8_t Key_Scan_Click_Double(uint8_t timeout_ticks);
uint8_t Key_Scan_Long_Press(void);

#endif