#include "sensors.h"

typedef struct
{
	float mean = 0;
	float variance = 0;
	uint64_t n = 0;
	uint64_t start_timestamp = 0;
	uint64_t last_timestamp = 0;
} statistics_t;

void ComputeStatisticsRecursive(statistics_t *stats, uint32_t period_ms, float value)
{
	if (stats->start_timestamp == 0)
	{
		stats->start_timestamp = HAL_GetTick();
	}

	stats->mean = (stats->mean * stats->n + value) / (stats->n + 1);
	stats->variance = (stats->variance * stats->n + pow(value - stats->mean, 2)) / (stats->n + 1);

	if (HAL_GetTick() - stats->start_timestamp < period_ms)
	{
		stats->n += 1;
	}

	// Print statistics once per second
	if (HAL_GetTick() - stats->last_timestamp >= 1000)
	{
		printf("Mean: %.8f, Variance: %.8f, STD (95%%): %.5f, Samples: %d\n", stats->mean, stats->variance, 2.0f * sqrt(stats->variance), stats->n);
		stats->last_timestamp = HAL_GetTick();
	}
}

extern osMutexId_t spi1MutexHandle;
extern osMutexId_t i2c1MutexHandle;

// LPS22HH

LPS22HH_Object_t lps22hh;

int32_t Write_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

int32_t Read_LPS22HH(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, BAR_CS_GPIO_Port, BAR_CS_Pin, reg, data, len);
}

BarData bar_data;

void StartBarTask(void *argument)
{
	bar_data.active = false;

	lps22hh.Ctx.handle = &hspi1;
	lps22hh.Ctx.write_reg = Write_LPS22HH;
	lps22hh.Ctx.read_reg = Read_LPS22HH;

	lps22hh.IO.BusType = LPS22HH_SPI_4WIRES_BUS;

	uint8_t id = 0, rst = 0;
	LPS22HH_ReadID(&lps22hh, &id);

	lps22hh_reset_set(&lps22hh.Ctx, PROPERTY_ENABLE);

	do
	{
		lps22hh_reset_get(&lps22hh.Ctx, &rst);
	} while (rst);

	LPS22HH_Init(&lps22hh);

	LPS22HH_PRESS_Enable(&lps22hh);
	LPS22HH_PRESS_SetOutputDataRate(&lps22hh, 200.0f);
	LPS22HH_Set_Filter_Mode(&lps22hh, LPS22HH_LPF_ODR_DIV_9);

	LPS22HH_TEMP_Enable(&lps22hh);

	osDelay(10);

	float pressure, temperature;

	float altitude_offset = 0;
	bool altitude_calibration = false;
	float status = false;

	statistics_t stats;

	while (true)
	{
		status = (LPS22HH_PRESS_GetPressure(&lps22hh, &pressure) == LPS22HH_OK) && (LPS22HH_TEMP_GetTemperature(&lps22hh, &temperature) == LPS22HH_OK);

		// Check if the barometer is active
		if (!status || id != LPS22HH_ID)
		{
			bar_data.active = false;

			osDelay(100);
			continue;
		}

		bar_data.pressure = pressure;
		bar_data.temperature = temperature;

		bar_data.altitude = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) + altitude_offset;

		if (!altitude_calibration && gps_data.reference_set)
		{
			altitude_offset = gps_data.reference.altitude - bar_data.altitude;
			altitude_calibration = true;
		}

		if (altitude_calibration)
		{
			bar_data.active = true;
		}

		// ComputeStatisticsRecursive(&stats, 1000, bar_altitude);
		// printf("ID: 0x%02X Pressure: %.2f, Altitude: %.2f, Temperature: %.2f, Mean %.3f, STD Deviation: %.5f (95%%), Samples/s: %d\n", id, bar_pressure, bar_altitude, bar_temperature, stats.mean, 2.0f * sqrt(stats.variance), stats.n);

		osDelay(10);
	}
}

// LIS3MDTR

MagData mag_data;

LIS3MDL_Object_t lis3mdl;

int32_t Write_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	reg |= 0x40; // Set the multi-write bit (0bx1xxxxxx)
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	reg |= 0x40; // Set the multi-read bit (0bx1xxxxxx)
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

void StartMagTask(void *argument)
{
	mag_data.active = false;

	lis3mdl.Ctx.handle = &hspi1;
	lis3mdl.Ctx.write_reg = Write_LIS3MDL;
	lis3mdl.Ctx.read_reg = Read_LIS3MDL;

	lis3mdl.IO.BusType = LIS3MDL_SPI_4WIRES_BUS;

	uint8_t id = 0, rst = 0;
	LIS3MDL_ReadID(&lis3mdl, &id);

	lis3mdl_reset_set(&lis3mdl.Ctx, PROPERTY_ENABLE);

	do
	{
		lis3mdl_reset_get(&lis3mdl.Ctx, &rst);
	} while (rst);

	LIS3MDL_Init(&lis3mdl);

	// Magnetometer configuration
	LIS3MDL_MAG_Enable(&lis3mdl);

	// Set the data rate to 155 Hz in ultra-high-performance mode
	lis3mdl_data_rate_set(&lis3mdl.Ctx, LIS3MDL_UHP_155Hz);

	// Set the full-scale to 4 Gauss
	LIS3MDL_MAG_SetFullScale(&lis3mdl, LIS3MDL_4_GAUSS);

	osDelay(10);

	LIS3MDL_Axes_t mag;
	bool status = false;

	while (true)
	{
		Read_LIS3MDL(&lis3mdl, 0x0F, &id, 1);

		status = LIS3MDL_MAG_GetAxes(&lis3mdl, &mag) == LIS3MDL_OK;

		// Check if the magnetometer is active
		if (!status || id != LIS3MDL_ID)
		{
			mag_data.active = false;

			osDelay(100);
			continue;
		}

		mag_data.magnetic_field = Vector{(float)mag.x, (float)mag.y, (float)mag.z};

		mag_data.active = true;

		// printf("ID: 0x%02X Magnetic Field: %.2f %.2f %.2f\n", id, *mag_data.magnetic_field.x, *mag_data.magnetic_field.y, *mag_data.magnetic_field.z);

		osDelay(50);
	}
}

// LSM6DSO

LSM6DSO_Object_t lsm6dso;

int32_t Write_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

int32_t Read_LSM6DSO(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, IMU_CS_GPIO_Port, IMU_CS_Pin, reg, data, len);
}

IMUData imu_data;

void StartImuTask(void *argument)
{
	imu_data.active = false;

	lsm6dso.Ctx.handle = &hspi1;
	lsm6dso.Ctx.write_reg = Write_LSM6DSO;
	lsm6dso.Ctx.read_reg = Read_LSM6DSO;
	lsm6dso.Ctx.mdelay = HAL_Delay;

	lsm6dso.IO.BusType = LSM6DSO_SPI_4WIRES_BUS;

	uint8_t id = 0, rst = 0;
	LSM6DSO_ReadID(&lsm6dso, &id);

	lsm6dso_reset_set(&lsm6dso.Ctx, PROPERTY_ENABLE);

	do
	{
		lsm6dso_reset_get(&lsm6dso.Ctx, &rst);
	} while (rst);

	LSM6DSO_Init(&lsm6dso);

	// Accelerometer configuration
	LSM6DSO_ACC_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_ACC_HIGH_PERFORMANCE_MODE);
	LSM6DSO_ACC_SetFullScale(&lsm6dso, 4);
	LSM6DSO_ACC_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);
	LSM6DSO_ACC_Enable(&lsm6dso);

	// Gyroscope configuration
	LSM6DSO_GYRO_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE);
	LSM6DSO_GYRO_SetFullScale(&lsm6dso, 500);
	LSM6DSO_GYRO_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);
	LSM6DSO_GYRO_Enable(&lsm6dso);

	osDelay(10);

	// Accelerometer and gyroscope scale factors
	float acc_scale = GRAVITY / 1000.0f;
	float gyro_scale = (M_PI / 180.0f) / 1000.0f;

	LSM6DSO_Axes_t acc, gyro;
	bool status = false;

	// Calibrate the IMU
	uint16_t samples = 1000;
	Vector acc_bias(3), gyro_bias(3);

	for (uint16_t i = 0; i < samples; i++)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		acc_bias += -Vector{(float)acc.x, (float)acc.y, (float)acc.z};
		gyro_bias += Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z};

		osDelay(2);
	}

	acc_bias *= (acc_scale / (float)samples);
	gyro_bias *= (gyro_scale / (float)samples);

	*acc_bias.z += GRAVITY; // Subtract gravity from the z-axis

	statistics_t stats;

	while (true)
	{
		status = (LSM6DSO_ACC_GetAxes(&lsm6dso, &acc) == LSM6DSO_OK) && (LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro) == LSM6DSO_OK);

		// Check if the IMU is active
		if (!status || id != LSM6DSO_ID)
		{
			imu_data.active = false;

			osDelay(100);
			continue;
		}

		// Map the IMU data to the IMU data structure
		imu_data.acceleration = (-Vector{(float)acc.x, (float)acc.y, (float)acc.z} * acc_scale - acc_bias);
		imu_data.angular_velocity = (Vector{(float)gyro.x, (float)gyro.y, (float)gyro.z} * gyro_scale - gyro_bias);

		imu_data.active = true;

		// ComputeStatisticsRecursive(&stats, 1000, *imu_data.angular_velocity.z);

		// Print biases
		// printf("ID: 0x%02X Acceleration: %.4f %.4f %.4f Gyroscope: %.4f %.4f %.4f\n", id, *acc_bias.x, *acc_bias.y, *acc_bias.z, *gyro_bias.x, *gyro_bias.y, *gyro_bias.z);

		// printf("ID: 0x%02X Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n", id, *imu_data.acceleration.x, *imu_data.acceleration.y, *imu_data.acceleration.z, *imu_data.angular_velocity.x, *imu_data.angular_velocity.y, *imu_data.angular_velocity.z);

		osDelay(2);
	}
}

GPSData gps_data;

void StartGpsTask(void *argument)
{
	gps_data.reference_set = false;
	gps_data.active = false;

	gps.init(&huart4);

	gps.setBaudRate(GPS_BAUDRATE_115200);
	gps.setOutputRate(GPS_ODR_10HZ);

	gps.start();

	osDelay(10);

	while (true)
	{
		if (gps.isFixed())
		{
			HAL_GPIO_TogglePin(LED2_STATUS2_PE1_GPIO_Port, LED2_STATUS2_PE1_Pin);
			HAL_GPIO_WritePin(LED3_STATUS3_PE2_GPIO_Port, LED3_STATUS3_PE2_Pin, GPIO_PIN_RESET);

			if (!gps_data.reference_set)
			{
				gps_data.reference.latitude = gps.getLatitude();
				gps_data.reference.longitude = gps.getLongitude();
				gps_data.reference.altitude = gps.getAltitude();

				gps_data.reference_set = true;
			}

			gps_data.coordinate.latitude = gps.getLatitude();
			gps_data.coordinate.longitude = gps.getLongitude();
			gps_data.coordinate.altitude = gps.getAltitude();

			gps.getVelocity(gps_data.velocity.x, gps_data.velocity.y);
			gps_data.orientation_z = gps.getOrientationZ();

			gps_data.active = true;

			// printf("Latitude: %.8f Longitude: %.8f Altitude: %.2f\n", gps_latitude, gps_longitude, gps_altitude);
			// printf("Speed (km/h): %.2f, Velocity: %.2f %.2f Orientation: %.5f\n", velocity.magnitude() * 3.6f, gps_velocity.x, gps_velocity.y, gps_orientation_z);
		}
		else
		{
			HAL_GPIO_WritePin(LED2_STATUS2_PE1_GPIO_Port, LED2_STATUS2_PE1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_STATUS3_PE2_GPIO_Port, LED3_STATUS3_PE2_Pin, GPIO_PIN_SET);

			gps_data.active = false;

			gps.start(); // Restart the GPS
		}

		osDelay(100);
	}
}

#define TF_LUNA_ADDRESS (0x10 << 1) // Address might be 0x10, shift for STM HAL

TOFData tof_data;

void StartTofTask(void *argument)
{
	tof_data.active = false;

	uint8_t data[2] = {0};
	bool status = false;

	osDelay(10);

	while (true)
	{
		memset(data, 0, sizeof(data));
		status = I2C_Read_Register(&hi2c1, &i2c1MutexHandle, TF_LUNA_ADDRESS, 0x00, data, 2) == HAL_OK;

		// Check if the TOF sensor is active
		if (!status)
		{
			tof_data.active = false;

			osDelay(100);
			continue;
		}

		tof_data.distance = ((data[1] << 8) | data[0]) / 100.0f;
		tof_data.active = true;

		// printf("Distance: %.2f\n", tof_distance);
		osDelay(50);
	}
}