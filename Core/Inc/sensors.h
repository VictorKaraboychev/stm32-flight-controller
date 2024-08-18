#ifndef __SENSORS_H__
#define __SENSORS_H__

#include "main.h"
#include "cmsis_os.h"

#include "lps22hh.h"
#include "lsm6dso.h"
#include "lis3mdl.h"
#include "gps.h"

#include "spi.h"
#include "i2c.h"

#include "vector.h"

void StartImuTask(void *argument);
void StartBarTask(void *argument);
void StartMagTask(void *argument);
void StartGpsTask(void *argument);
void StartTofTask(void *argument);

extern volatile Vector3 imu_acceleration;
extern volatile Vector3 imu_angular_velocity;
extern volatile Vector3 mag_magnetic_field;

extern volatile float bar_temperature;
extern volatile float bar_pressure;
extern volatile float bar_altitude;

extern volatile float gps_latitude;
extern volatile float gps_longitude;
extern volatile float gps_altitude;

extern volatile float tof_distance;

#endif /* __SENSORS_H__ */