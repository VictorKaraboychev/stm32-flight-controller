#include "control.h"

void StartStatusLedTask(void *argument)
{
	while (true)
	{
		HAL_GPIO_TogglePin(LED1_STATUS1_PE0_GPIO_Port, LED1_STATUS1_PE0_Pin);
		osDelay(500);
	}
}

void StartFusionTask(void *argument)
{
	while (true)
	{
		osDelay(500);
	}
}

void StartControlTask(void *argument)
{
	while (true)
	{
		osDelay(500);
	}
}