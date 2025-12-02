/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
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
#include "i2c.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

I2C_HandleTypeDef hi2c2;

/* I2C2 init function */
void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */

  /* USER CODE END I2C2_MspInit 0 */

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();

    /**I2C2 GPIO Configuration
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);

    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);

  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* =================================================================================
   MPU6050 / DMP 兼容层接口实现
   ================================================================================= */

#define I2C_TIMEOUT  1000

// 兼容 DMP 库：写寄存器 (地址左移适配HAL)
int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

// 兼容 DMP 库：读寄存器 (地址左移适配HAL)
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, (addr << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, len, I2C_TIMEOUT);
    return (status == HAL_OK) ? 0 : 1;
}

// 兼容 MPU6050.c：写多个字节 (dev是8位地址)
u8 IICwriteBytes(u8 dev, u8 reg, u8 length, u8* data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Write(&hi2c2, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);
    return (status == HAL_OK) ? 1 : 0;
}

// 兼容 MPU6050.c：读多个字节 (dev是8位地址)
u8 IICreadBytes(u8 dev, u8 reg, u8 length, u8 *data)
{
    HAL_StatusTypeDef status;
    status = HAL_I2C_Mem_Read(&hi2c2, dev, reg, I2C_MEMADD_SIZE_8BIT, data, length, I2C_TIMEOUT);
    return (status == HAL_OK) ? length : 0;
}

// 写单个字节
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data)
{
    return IICwriteBytes(dev, reg, 1, &data);
}

// 读单个字节
u8 IICreadByte(u8 dev, u8 reg, u8 *data)
{
    return (IICreadBytes(dev, reg, 1, data) > 0) ? 1 : 0;
}

// 读位操作：读-改-写
u8 IICwriteBits(u8 dev, u8 reg, u8 bitStart, u8 length, u8 data)
{
    u8 b;
    if (IICreadByte(dev, reg, &b) != 0) {
        u8 mask = (0xFF << (bitStart + 1)) | 0xFF >> ((8 - bitStart) + length - 1);
        data <<= (8 - length);
        data >>= (7 - bitStart);
        b &= mask;
        b |= data;
        return IICwriteByte(dev, reg, b);
    } else {
        return 0;
    }
}

// 写单个位
u8 IICwriteBit(u8 dev, u8 reg, u8 bitNum, u8 data)
{
    u8 b;
    IICreadByte(dev, reg, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return IICwriteByte(dev, reg, b);
}

// 读取单个字节并返回 (Main.c用)
unsigned char I2C_ReadOneByte(unsigned char I2C_Addr, unsigned char addr)
{
    unsigned char val = 0;
    HAL_I2C_Mem_Read(&hi2c2, I2C_Addr, addr, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT);
    return val;
}
/* USER CODE END 1 */
