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
#include "coordinate.h"

#define GRAVITY 9.81f

struct IMUData
{
	Vector acceleration;
	Vector angular_velocity;

	bool active;
};

struct BarData
{
	float temperature;
	float pressure;
	float altitude;

	bool active;
};

struct MagData
{
	Vector magnetic_field;

	bool active;
};

struct GPSData
{
	Coordinate coordinate;
	Coordinate reference;

	Vector velocity;
	float orientation_z;
	bool reference_set;
	bool active;
};

struct TOFData
{
	float distance;

	bool active;
};

extern IMUData imu_data;
extern BarData bar_data;
extern MagData mag_data;
extern GPSData gps_data;
extern TOFData tof_data;

void StartImuTask(void *argument);
void StartBarTask(void *argument);
void StartMagTask(void *argument);
void StartGpsTask(void *argument);
void StartTofTask(void *argument);

#endif /* __SENSORS_H__ */