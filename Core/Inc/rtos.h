#ifndef RTOS_H
#define RTOS_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

	extern osMutexId_t spi1MutexHandle;
	extern osMutexId_t i2c1MutexHandle;
	extern osMutexId_t i2c2MutexHandle;
	extern osMutexId_t usbMutexHandle;
	extern osMutexId_t fdcanMutexHandle;

	void MX_FREERTOS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_H */