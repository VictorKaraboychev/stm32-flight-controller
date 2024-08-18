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
#include "control.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <usb_device.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

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
	.stack_size = 128 * 4,
	.priority = (osPriority_t)osPriorityLow,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
	.name = "gpsTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
	.name = "imuTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for magTask */
osThreadId_t magTaskHandle;
const osThreadAttr_t magTask_attributes = {
	.name = "magTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for barTask */
osThreadId_t barTaskHandle;
const osThreadAttr_t barTask_attributes = {
	.name = "barTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for fusionTask */
osThreadId_t fusionTaskHandle;
const osThreadAttr_t fusionTask_attributes = {
	.name = "fusionTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
	.name = "controlTask",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t)osPriorityRealtime,
};
/* Definitions for tofTask */
osThreadId_t tofTaskHandle;
const osThreadAttr_t tofTask_attributes = {
	.name = "tofTask",
	.stack_size = 512 * 4,
	.priority = (osPriority_t)osPriorityNormal,
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
extern void StartStatusLedTask(void *argument);
extern void StartGpsTask(void *argument);
extern void StartImuTask(void *argument);
extern void StartMagTask(void *argument);
extern void StartBarTask(void *argument);
extern void StartFusionTask(void *argument);
extern void StartControlTask(void *argument);
extern void StartTofTask(void *argument);

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

	/* creation of gpsTask */
	gpsTaskHandle = osThreadNew(StartGpsTask, NULL, &gpsTask_attributes);

	/* creation of imuTask */
	imuTaskHandle = osThreadNew(StartImuTask, NULL, &imuTask_attributes);

	/* creation of magTask */
	magTaskHandle = osThreadNew(StartMagTask, NULL, &magTask_attributes);

	/* creation of barTask */
	barTaskHandle = osThreadNew(StartBarTask, NULL, &barTask_attributes);

	/* creation of fusionTask */
	fusionTaskHandle = osThreadNew(StartFusionTask, NULL, &fusionTask_attributes);

	/* creation of controlTask */
	controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);

	/* creation of tofTask */
	tofTaskHandle = osThreadNew(StartTofTask, NULL, &tofTask_attributes);

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
	MX_USB_DEVICE_Init();
	/* USER CODE BEGIN StartDefaultTask */

	/* Infinite loop */
	while (1)
	{
		osDelay(1);
	}
	/* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
