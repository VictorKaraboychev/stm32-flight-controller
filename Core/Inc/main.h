/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

	/* Private includes ----------------------------------------------------------*/
	/* USER CODE BEGIN Includes */

	/* USER CODE END Includes */

	/* Exported types ------------------------------------------------------------*/
	/* USER CODE BEGIN ET */

	/* USER CODE END ET */

	/* Exported constants --------------------------------------------------------*/
	/* USER CODE BEGIN EC */

	/* USER CODE END EC */

	/* Exported macro ------------------------------------------------------------*/
	/* USER CODE BEGIN EM */

	/* USER CODE END EM */

	/* Exported functions prototypes ---------------------------------------------*/
	void Error_Handler(void);

	/* USER CODE BEGIN EFP */

	int _write(int file, char *ptr, int length);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED3_STATUS3_PE2_Pin GPIO_PIN_2
#define LED3_STATUS3_PE2_GPIO_Port GPIOE
#define LED4_STATUS4_PE3_Pin GPIO_PIN_3
#define LED4_STATUS4_PE3_GPIO_Port GPIOE
#define LED5_PWR1_PE4_Pin GPIO_PIN_4
#define LED5_PWR1_PE4_GPIO_Port GPIOE
#define LED6_PWR2_PE5_Pin GPIO_PIN_5
#define LED6_PWR2_PE5_GPIO_Port GPIOE
#define LED7_PWR3_PE6_Pin GPIO_PIN_6
#define LED7_PWR3_PE6_GPIO_Port GPIOE
#define PWM1_PE8_Pin GPIO_PIN_8
#define PWM1_PE8_GPIO_Port GPIOE
#define PWM2_PE9_Pin GPIO_PIN_9
#define PWM2_PE9_GPIO_Port GPIOE
#define PWM3_PE10_Pin GPIO_PIN_10
#define PWM3_PE10_GPIO_Port GPIOE
#define PWM4_PE11_Pin GPIO_PIN_11
#define PWM4_PE11_GPIO_Port GPIOE
#define PWM5_PE12_Pin GPIO_PIN_12
#define PWM5_PE12_GPIO_Port GPIOE
#define PWM6_PE13_Pin GPIO_PIN_13
#define PWM6_PE13_GPIO_Port GPIOE
#define PWM7_PE14_Pin GPIO_PIN_14
#define PWM7_PE14_GPIO_Port GPIOE
#define PWM8_PE15_Pin GPIO_PIN_15
#define PWM8_PE15_GPIO_Port GPIOE
#define GPIO_PD8_Pin GPIO_PIN_8
#define GPIO_PD8_GPIO_Port GPIOD
#define GPIO_PD9_Pin GPIO_PIN_9
#define GPIO_PD9_GPIO_Port GPIOD
#define GPIO_PD10_Pin GPIO_PIN_10
#define GPIO_PD10_GPIO_Port GPIOD
#define GPIO_PD11_Pin GPIO_PIN_11
#define GPIO_PD11_GPIO_Port GPIOD
#define GPIO_PD12_Pin GPIO_PIN_12
#define GPIO_PD12_GPIO_Port GPIOD
#define GPIO_PD13_Pin GPIO_PIN_13
#define GPIO_PD13_GPIO_Port GPIOD
#define GPIO_PD14_Pin GPIO_PIN_14
#define GPIO_PD14_GPIO_Port GPIOD
#define GPIO_PD15_Pin GPIO_PIN_15
#define GPIO_PD15_GPIO_Port GPIOD
#define BAR_DDRY_Pin GPIO_PIN_12
#define BAR_DDRY_GPIO_Port GPIOC
#define GPIO_PD0_Pin GPIO_PIN_0
#define GPIO_PD0_GPIO_Port GPIOD
#define GPIO_PD1_Pin GPIO_PIN_1
#define GPIO_PD1_GPIO_Port GPIOD
#define GPIO_PD2_Pin GPIO_PIN_2
#define GPIO_PD2_GPIO_Port GPIOD
#define GPIO_PD3_Pin GPIO_PIN_3
#define GPIO_PD3_GPIO_Port GPIOD
#define GPIO_PD4_Pin GPIO_PIN_4
#define GPIO_PD4_GPIO_Port GPIOD
#define GPIO_PD5_Pin GPIO_PIN_5
#define GPIO_PD5_GPIO_Port GPIOD
#define GPIO_PD6_Pin GPIO_PIN_6
#define GPIO_PD6_GPIO_Port GPIOD
#define GPIO_PD7_Pin GPIO_PIN_7
#define GPIO_PD7_GPIO_Port GPIOD
#define IMU_INT1_Pin GPIO_PIN_9
#define IMU_INT1_GPIO_Port GPIOG
#define IMU_INT2_Pin GPIO_PIN_10
#define IMU_INT2_GPIO_Port GPIOG
#define SD_CS_Pin GPIO_PIN_11
#define SD_CS_GPIO_Port GPIOG
#define BAR_CS_Pin GPIO_PIN_12
#define BAR_CS_GPIO_Port GPIOG
#define MAG_CS_Pin GPIO_PIN_13
#define MAG_CS_GPIO_Port GPIOG
#define IMU_CS_Pin GPIO_PIN_14
#define IMU_CS_GPIO_Port GPIOG
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOG
#define MAG_DDRY_Pin GPIO_PIN_8
#define MAG_DDRY_GPIO_Port GPIOB
#define LED1_STATUS1_PE0_Pin GPIO_PIN_0
#define LED1_STATUS1_PE0_GPIO_Port GPIOE
#define LED2_STATUS2_PE1_Pin GPIO_PIN_1
#define LED2_STATUS2_PE1_GPIO_Port GPIOE

	/* USER CODE BEGIN Private defines */
	__weak void MX_FREERTOS_Init(void);

	/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
