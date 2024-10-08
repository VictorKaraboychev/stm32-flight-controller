/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    spi.c
 * @brief   This file provides code for the configuration
 *          of the SPI instances.
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
/* Includes ------------------------------------------------------------------*/
#include "spi.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x0;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (spiHandle->Instance == SPI1)
	{
		/* USER CODE BEGIN SPI1_MspInit 0 */

		/* USER CODE END SPI1_MspInit 0 */
		/* SPI1 clock enable */
		__HAL_RCC_SPI1_CLK_ENABLE();

		__HAL_RCC_GPIOB_CLK_ENABLE();
		/**SPI1 GPIO Configuration
		PB3 (JTDO/TRACESWO)     ------> SPI1_SCK
		PB4 (NJTRST)     ------> SPI1_MISO
		PB5     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* USER CODE BEGIN SPI1_MspInit 1 */

		/* USER CODE END SPI1_MspInit 1 */
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{

	if (spiHandle->Instance == SPI1)
	{
		/* USER CODE BEGIN SPI1_MspDeInit 0 */

		/* USER CODE END SPI1_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PB3 (JTDO/TRACESWO)     ------> SPI1_SCK
		PB4 (NJTRST)     ------> SPI1_MISO
		PB5     ------> SPI1_MOSI
		*/
		HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

		/* USER CODE BEGIN SPI1_MspDeInit 1 */

		/* USER CODE END SPI1_MspDeInit 1 */
	}
}

/* USER CODE BEGIN 1 */

HAL_StatusTypeDef SPI_Read_Register(SPI_HandleTypeDef *hspi, osMutexId_t *mspi, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, uint8_t *pData, uint16_t size)
{
	// Acquire the SPI mutex
	osMutexAcquire(*mspi, osWaitForever);

	uint8_t buffer[size + 1];

	// The register address needs to be OR-ed with the READ_MASK to set the MSB (read command)
	buffer[0] = regAddr | READ_MASK;

	// Initiate the SPI transmission
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
	uint8_t status = HAL_SPI_TransmitReceive(hspi, buffer, buffer, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	// Check if the transmission was successful
	if (status != HAL_OK)
	{
		// Release the SPI mutex
		osMutexRelease(*mspi);

		return HAL_ERROR;
	}

	// Store the received data in pData
	for (int i = 0; i < size; i++)
	{
		pData[i] = buffer[i + 1]; // Skip the first byte (dummy byte)
	}

	// Release the SPI mutex
	osMutexRelease(*mspi);

	return HAL_OK;
}

HAL_StatusTypeDef SPI_Write_Register(SPI_HandleTypeDef *hspi, osMutexId_t *mspi, GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, const uint8_t *pData, uint16_t size)
{
	// Acquire the SPI mutex
	osMutexAcquire(*mspi, osWaitForever);

	uint8_t buffer[size + 1];

	// The register address needs to be OR-ed with the WRITE_MASK to indicate a write operation
	buffer[0] = regAddr & WRITE_MASK;

	// Copy the data to be written into the txBuffer
	for (int i = 0; i < size; i++)
	{
		buffer[i + 1] = pData[i];
	}

	// Initiate the SPI transmission
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
	uint8_t status = HAL_SPI_Transmit(hspi, buffer, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	// Release the SPI mutex
	osMutexRelease(*mspi);

	// Check if the transmission was successful
	if (status != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

/* USER CODE END 1 */
