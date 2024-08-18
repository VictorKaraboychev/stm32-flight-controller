#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"

#include "sensors.h"

void StartStatusLedTask(void *argument);
void StartFusionTask(void *argument);
void StartControlTask(void *argument);


#endif /* __CONTROL_H__ */