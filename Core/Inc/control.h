#ifndef __CONTROL_H__
#define __CONTROL_H__

#include "main.h"
#include "cmsis_os.h"

#include "vector.h"
#include "matrix.h"
#include "extended_kalman_filter.h"

#include "sensors.h"

void StartStatusLedTask(void *argument);
void StartFusionTask(void *argument);
void StartControlTask(void *argument);


#endif /* __CONTROL_H__ */