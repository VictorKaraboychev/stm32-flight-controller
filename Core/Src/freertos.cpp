/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "lps22hh.h"
// #include "lis3mdl.h"
#include "lsm6dso.h"
#include "spi.h"
#include "gps.h"
#include "usart.h"
#include "main.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define READ_MASK 0x80
#define WRITE_MASK 0x7F

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

LPS22HH_Object_t lps22hh;
// LIS3MDTR_Object_t lis3mdl;
LSM6DSO_Object_t lsm6dso;

// void lps22hhInit();
// float pressureToAltitude(float pressure);
// float getPressure();
// float getTemperature();

// void lsm6dsoInit();
// void getAcceleration(float *x, float *y, float *z);
// void getGyroscope(float *x, float *y, float *z);

static uint8_t Read_SPI_Register(GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, uint8_t *pData, uint16_t size);
static uint8_t Write_SPI_Register(GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, const uint8_t *pData, uint16_t size);

// HAL_StatusTypeDef Init_LPS22HH(void);
int32_t Write_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t Read_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

int32_t Write_LIS3MDTR(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t Read_LIS3MDTR(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

// HAL_StatusTypeDef Init_LSM6DSO(void);
int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len);
int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len);

// typedef int32_t (*stmdev_write_ptr)(void *, uint8_t, uint8_t *, uint16_t);
// typedef int32_t (*stmdev_read_ptr)(void *, uint8_t, uint8_t *, uint16_t);

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for statusLedTask */
osThreadId_t statusLedTaskHandle;
const osThreadAttr_t statusLedTask_attributes = {
	.name = "statusLedTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t)osPriorityLow,
};
/* Definitions for spi1Mutex */
osMutexId_t spi1MutexHandle;
const osMutexAttr_t spi1Mutex_attributes = {
	.name = "spi1Mutex"};
/* Definitions for i2c1Mutex */
osMutexId_t i2c1MutexHandle;
const osMutexAttr_t i2c1Mutex_attributes = {
	.name = "i2c1Mutex"};
/* Definitions for i2c2Mutex */
osMutexId_t i2c2MutexHandle;
const osMutexAttr_t i2c2Mutex_attributes = {
	.name = "i2c2Mutex"};
/* Definitions for usbMutex */
osMutexId_t usbMutexHandle;
const osMutexAttr_t usbMutex_attributes = {
	.name = "usbMutex"};
/* Definitions for fdcanMutex */
osMutexId_t fdcanMutexHandle;
const osMutexAttr_t fdcanMutex_attributes = {
	.name = "fdcanMutex"};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartStatusLedTask(void *argument);

extern "C"
{
	extern void MX_USB_DEVICE_Init(void);
	void MX_FREERTOS_Init(void);
}

/**
 * @brief  FreeRTOS initialization
 * @param  None
 * @retval None
 */
void MX_FREERTOS_Init(void)
{
	/* USER CODE BEGIN Init */

	/* USER CODE END Init */
	/* Create the mutex(es) */
	/* creation of spi1Mutex */
	spi1MutexHandle = osMutexNew(&spi1Mutex_attributes);

	/* creation of i2c1Mutex */
	i2c1MutexHandle = osMutexNew(&i2c1Mutex_attributes);

	/* creation of i2c2Mutex */
	i2c2MutexHandle = osMutexNew(&i2c2Mutex_attributes);

	/* creation of usbMutex */
	usbMutexHandle = osMutexNew(&usbMutex_attributes);

	/* creation of fdcanMutex */
	fdcanMutexHandle = osMutexNew(&fdcanMutex_attributes);

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
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of statusLedTask */
	statusLedTaskHandle = osThreadNew(StartStatusLedTask, NULL, &statusLedTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
	/* init code for USB_DEVICE */
	/* USER CODE BEGIN StartDefaultTask */
	/* Infinite loop */

	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(BAR_CS_GPIO_Port, BAR_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MAG_CS_GPIO_Port, MAG_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	lps22hh.Ctx.handle = &hspi1;
	lps22hh.Ctx.write_reg = Write_LPS22HH;
	lps22hh.Ctx.read_reg = Read_LPS22HH;

	LPS22HH_Init(&lps22hh);

	LPS22HH_PRESS_Enable(&lps22hh);
	LPS22HH_PRESS_SetOutputDataRate(&lps22hh, 200.0f);

	LPS22HH_TEMP_Enable(&lps22hh);

	lsm6dso.Ctx.handle = &hspi1;
	lsm6dso.Ctx.write_reg = Write_LSM6DSO;
	lsm6dso.Ctx.read_reg = Read_LSM6DSO;

	LSM6DSO_Init(&lsm6dso);

	LSM6DSO_ACC_Enable(&lsm6dso);
	LSM6DSO_ACC_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
	LSM6DSO_ACC_SetFullScale(&lsm6dso, LSM6DSO_4g);
	LSM6DSO_ACC_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);

	LSM6DSO_GYRO_Enable(&lsm6dso);
	LSM6DSO_GYRO_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE);
	LSM6DSO_GYRO_SetFullScale(&lsm6dso, LSM6DSO_500dps);

	osDelay(10);

	// float avgPressure = 0.0f;

	// for (int i = 0; i < 50; i++)
	// {
	// 	float p;
	// 	LPS22HH_PRESS_GetPressure(&lps22hh, &p);
	// 	avgPressure += p;
	// 	osDelay(5);
	// }

	// avgPressure /= 50.0f;

	// float referenceAltitude = pressureToAltitude(avgPressure);

	// float a = 0.05f;
	// float pressure = 0.0f;

	while (1)
	{
		float pressure, temperature;
		LPS22HH_PRESS_GetPressure(&lps22hh, &pressure);
		LPS22HH_TEMP_GetTemperature(&lps22hh, &temperature);

		// printf("Pressure: %.2f, Temperature: %.2f\n", pressure, temperature);

		// pressure = pressure * (1 - a) + a * getPressure();

		// float temperature = getTemperature();
		// float altitude = pressureToAltitude(pressure);

		// printf("Pressure: %.2f Pa Temperature: %.2f C  Altitude: %.4f m \n", pressure, temperature, altitude - referenceAltitude);

		// float ax, ay, az;
		// getAcceleration(&ax, &ay, &az);

		// float gx, gy, gz;
		// getGyroscope(&gx, &gy, &gz);

		// printf("Acceleration: %.2f %.2f %.2f Gyroscope: %.2f %.2f %.2f\n", ax, ay, az, gx, gy, gz);

		// LSM6DSO_AxesRaw_t  acc, gyro;

		// LSM6DSO_ACC_GetAxesRaw(&lsm6dso, &acc);
		// LSM6DSO_ACC_GetAxesRaw(&lsm6dso, &gyro);

		// printf("Acceleration: %d %d %d Gyroscope: %d %d %d\n", (int)acc.x, (int)acc.y, (int)acc.z, (int)gyro.x, (int)gyro.y, (int)gyro.z);

		osDelay(5);
	}
	/* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartStatusLedTask */
/**
 * @brief Function implementing the statusLedTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartStatusLedTask */
void StartStatusLedTask(void *argument)
{
	/* USER CODE BEGIN StartStatusLedTask */
	gps.init(&huart4);

	gps.setBaudRate(GPS_BAUDRATE_115200);
	gps.setOutputRate(GPS_ODR_10HZ);

	gps.start();

	/* Infinite loop */
	while (1)
	{
		HAL_GPIO_TogglePin(LED1_STATUS1_PE0_GPIO_Port, LED1_STATUS1_PE0_Pin);

		printf("Latitude: %f Longitude: %f Altitude: %f\n", gps.getLatitude(), gps.getLongitude(), gps.getAltitude());

		osDelay(500);
	}
	/* USER CODE END StartStatusLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// void lsm6dsoInit()
// {
// 	lsm6dso_write(0x10, 0b00000101, 1); // CTRL1_XL
// 	lsm6dso_write(0x11, 0b10100100, 1); // CTRL2_G
// 	lsm6dso_write(0x12, 0b10000100, 1); // CTRL3_C
// }

// void getAcceleration(float *x, float *y, float *z)
// {
// 	float conversion = (0.122f / 1000.0f) * 9.81f; // m/s^2/LSB

// 	uint8_t d[6];
// 	lsm6dso_read(0x28, d, 6);

// 	*x = (int16_t)(d[1] << 8 | d[0]) * conversion;
// 	*y = (int16_t)(d[3] << 8 | d[2]) * conversion;
// 	*z = (int16_t)(d[5] << 8 | d[4]) * conversion;
// }

// void getGyroscope(float *x, float *y, float *z)
// {
// 	float conversion = (17.5f / 1000.0f) * (180.0f / 3.14159265f); // rad/s/LSB

// 	uint8_t d[6];
// 	lsm6dso_read(0x22, d, 6);

// 	*x = (int16_t)(d[1] << 8 | d[0]) * conversion;
// 	*y = (int16_t)(d[3] << 8 | d[2]) * conversion;
// 	*z = (int16_t)(d[5] << 8 | d[4]) * conversion;
// }

// void lps22hhInit()
// {
// 	lps22hh_write(0x10, 0b01110000, 1); // CTRL_REG1
// 										// lps22h_write(0x11, 0b00010010, 1); // CTRL_REG2
// }

// float pressureToAltitude(float pressure)
// {
// 	return 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255));
// }

// float getPressure()
// {
// 	uint8_t d[3];
// 	lps22hh_read(0x28, d, 3);

// 	return (d[2] << 16 | d[1] << 8 | d[0]) / 4096.0f;
// }

// float getTemperature()
// {
// 	uint8_t d[2];
// 	lps22hh_read(0x2B, d, 2);

// 	return (d[1] << 8 | d[0]) / 100.0f;
// }

static uint8_t Read_SPI_Register(GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, uint8_t *pData, uint16_t size)
{
	uint8_t buffer[size + 1];

	// The register address needs to be OR-ed with the READ_MASK to set the MSB (read command)
	buffer[0] = regAddr | READ_MASK;

	// Initiate the SPI transmission
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
	uint8_t status = HAL_SPI_TransmitReceive(&hspi1, buffer, buffer, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	// Check if the transmission was successful
	if (status != HAL_OK)
	{
		return HAL_ERROR;
	}

	// Store the received data in pData
	for (int i = 0; i < size; i++)
	{
		pData[i] = buffer[i + 1]; // Skip the first byte (dummy byte)
	}

	return HAL_OK;
}

static uint8_t Write_SPI_Register(GPIO_TypeDef *csPort, uint16_t csPin, uint8_t regAddr, const uint8_t *pData, uint16_t size)
{
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
	uint8_t status = HAL_SPI_Transmit(&hspi1, buffer, size + 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);

	// Check if the transmission was successful
	if (status != HAL_OK)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

// LPS22HH Functions

// HAL_StatusTypeDef Init_LPS22HH(void)
// {
// 	uint8_t id, rst = 0;

// 	/* Check device ID for Pressure sensor */
// 	lps22hh_device_id_get(&lps22hh, &id);
// 	/* Configure Pressure sensor */
// 	if (id == LPS22HH_ID)
// 	{
// 		/* Restore default configuration */
// 		lps22hh_reset_set(&lps22hh, PROPERTY_ENABLE);
// 		do
// 		{
// 			lps22hh_reset_get(&lps22hh, &rst);
// 		} while (rst);
// 		/* Enable Block Data Update */
// 		lps22hh_block_data_update_set(&lps22hh, PROPERTY_ENABLE);
// 		/* Set Output Data Rate */
// 		lps22hh_data_rate_set(&lps22hh, LPS22HH_200_Hz);
// 		return HAL_OK;
// 	}
// 	else
// 	{
// 		return HAL_ERROR;
// 	}
// }

int32_t Write_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Write_SPI_Register(BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

int32_t Read_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Read_SPI_Register(BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

// LIS3MDTR Functions

int32_t Write_LIS3MDTR(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Write_SPI_Register(MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS3MDTR(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Read_SPI_Register(MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

// LSM6DSO Functions

// HAL_StatusTypeDef Init_LSM6DSO(void)
// {
// 	uint8_t id, rst = 0;

// 	/* Check device ID for IMU sensor */
// 	uint8_t id;
// 	lsm6dso_reset_set(&lsm6dso, PROPERTY_ENABLE);
// 	/* Configure IMU sensor */
// 	if (id == LSM6DSO_ID)
// 	{
// 		/* Restore default configuration */
// 		lsm6dso_reset_set(&lsm6dso, PROPERTY_ENABLE);

// 		do
// 		{
// 			lsm6dso_reset_get(&lsm6dso, &rst);
// 		} while (rst);

// 		/* Disable I3C interface */
// 		lsm6dso_i3c_disable_set(&lsm6dso, LSM6DSO_I3C_DISABLE);

// 		/* Set Output Data Rate */
// 		lsm6dso_xl_data_rate_set(&lsm6dso, LSM6DSO_XL_ODR_6667Hz);
// 		lsm6dso_gy_data_rate_set(&lsm6dso, LSM6DSO_GY_ODR_6667Hz);

// 		/* Set full scale */
// 		lsm6dso_xl_full_scale_set(&lsm6dso, LSM6DSO_4g);
// 		lsm6dso_gy_full_scale_set(&lsm6dso, LSM6DSO_500dps);

// 		/* Configure filtering chain */
// 		lsm6dso_xl_hp_path_on_out_set(&lsm6dso, LSM6DSO_LP_ODR_DIV_10);
// 		lsm6dso_xl_filter_lp2_set(&lsm6dso, PROPERTY_ENABLE);

// 		return HAL_OK;
// 	}
// 	else
// 	{
// 		return HAL_ERROR;
// 	}
// }

int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Write_SPI_Register(IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return Read_SPI_Register(IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

/* USER CODE END Application */
