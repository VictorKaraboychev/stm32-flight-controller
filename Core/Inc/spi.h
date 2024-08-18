/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN Private defines */

#define READ_MASK 0x80
#define WRITE_MASK 0x7F

/* USER CODE END Private defines */

void MX_SPI1_Init(void);

/* USER CODE BEGIN Prototypes */

HAL_StatusTypeDef SPI_Read_Register(SPI_HandleTypeDef *hspi, osMutexId_t *mspi, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, uint8_t *pData, uint16_t size);
HAL_StatusTypeDef SPI_Write_Register(SPI_HandleTypeDef *hspi, osMutexId_t *mspi, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, const uint8_t *pData, uint16_t size);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

