/**
  ******************************************************************************
  * @file    custom_mems_conf.h
  * @author  MEMS Software Solutions Team
  * @brief   This file contains definitions of the MEMS components bus interfaces for custom boards
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_MEMS_CONF_H
#define CUSTOM_MEMS_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "custom_bus.h"
#include "custom_errno.h"

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#define USE_CUSTOM_MOTION_SENSOR_LIS3MDL_0        0U

#define USE_CUSTOM_MOTION_SENSOR_LSM6DSO_0        0U

#define USE_CUSTOM_ENV_SENSOR_LPS22HH_0           0U

#define CUSTOM_LIS3MDL_0_CS_PORT GPIOG
#define CUSTOM_LIS3MDL_0_CS_PIN GPIO_PIN_13

#define CUSTOM_LSM6DSO_0_CS_PORT GPIOG
#define CUSTOM_LSM6DSO_0_CS_PIN GPIO_PIN_14

#define CUSTOM_LPS22HH_0_CS_PORT GPIOG
#define CUSTOM_LPS22HH_0_CS_PIN GPIO_PIN_12

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_MEMS_CONF_H*/
