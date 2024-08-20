#include "sensors.h"

typedef struct
{
	float mean = 0;
	float variance = 0;
	uint64_t n = 0;
	uint64_t timestamp = 0;
} statistics_t;

void ComputeStatisticsRecursive(statistics_t *stats, uint32_t period_ms, float value)
{
	if (stats->timestamp == 0)
	{
		stats->timestamp = HAL_GetTick();
	}

	stats->mean = (stats->mean * stats->n + value) / (stats->n + 1);
	stats->variance = (stats->variance * stats->n + pow(value - stats->mean, 2)) / (stats->n + 1);

	if (HAL_GetTick() - stats->timestamp < period_ms)
	{
		stats->n += 1;
	}

	printf("Mean: %.3f, Variance: %.5f, STD (95%%): %.5f, Samples: %d\n", stats->mean, stats->variance, 2.0f * sqrt(stats->variance), stats->n);
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

volatile float bar_temperature;
volatile float bar_pressure;
volatile float bar_altitude;

void StartBarTask(void *argument)
{
	lps22hh.Ctx.handle = &hspi1;
	lps22hh.Ctx.write_reg = Write_LPS22HH;
	lps22hh.Ctx.read_reg = Read_LPS22HH;

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

	statistics_t stats;

	while (true)
	{
		LPS22HH_PRESS_GetPressure(&lps22hh, &pressure);
		LPS22HH_TEMP_GetTemperature(&lps22hh, &temperature);

		bar_pressure = pressure;
		bar_temperature = temperature;

		bar_altitude = 44330 * (1 - pow(pressure / 1013.25, 1 / 5.255)) + altitude_offset;

		if (!altitude_calibration && gps_reference_altitude != 0)
		{
			altitude_offset = gps_reference_altitude - bar_altitude;
			altitude_calibration = true;
		}

		// ComputeStatisticsRecursive(&stats, 1000, bar_altitude);
		// printf("ID: 0x%02X Pressure: %.2f, Altitude: %.2f, Temperature: %.2f, Mean %.3f, STD Deviation: %.5f (95%%), Samples/s: %d\n", id, bar_pressure, bar_altitude, bar_temperature, stats.mean, 2.0f * sqrt(stats.variance), stats.n);

		osDelay(10);
	}
}

// LIS3MDTR

volatile Vector3 mag_magnetic_field;

LIS3MDL_Object_t lis3mdl;

int32_t Write_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Write_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

int32_t Read_LIS3MDL(void *handle, uint8_t reg, uint8_t *data, uint16_t len)
{
	return SPI_Read_Register(&hspi1, &spi1MutexHandle, MAG_CS_GPIO_Port, MAG_CS_Pin, reg, data, len);
}

void StartMagTask(void *argument)
{
	lis3mdl.Ctx.handle = &hspi1;
	lis3mdl.Ctx.write_reg = Write_LIS3MDL;
	lis3mdl.Ctx.read_reg = Read_LIS3MDL;

	uint8_t id = 0, rst = 0;
	LIS3MDL_ReadID(&lis3mdl, &id);

	lis3mdl_reset_set(&lis3mdl.Ctx, PROPERTY_ENABLE);

	do
	{
		lis3mdl_reset_get(&lis3mdl.Ctx, &rst);
	} while (rst);


	// LIS3MDL_Init(&lis3mdl);

	// LIS3MDL_MAG_Enable(&lis3mdl);
	// LIS3MDL_MAG_SetOutputDataRate(&lis3mdl, 100.0f);
	// LIS3MDL_MAG_SetFullScale(&lis3mdl, LIS3MDL_4_GAUSS);

	// osDelay(10);

	// LIS3MDL_Axes_t mag;

	while (true)
	{
		// LIS3MDL_MAG_GetAxes(&lis3mdl, &mag);

		// magnetic_field.x = mag.x;
		// magnetic_field.y = mag.y;
		// magnetic_field.z = mag.z;

		// printf("ID: 0x%02X Magnetic Field: %.2f %.2f %.2f\n", id, mag_magnetic_field.x, mag_magnetic_field.y, mag_magnetic_field.z);

		osDelay(500);
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

volatile Vector3 imu_acceleration;
volatile Vector3 imu_angular_velocity;

void StartImuTask(void *argument)
{
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
	uint8_t acc_status = LSM6DSO_ACC_Enable(&lsm6dso);

	// Gyroscope configuration
	LSM6DSO_GYRO_SetOutputDataRate_With_Mode(&lsm6dso, 6667.0f, LSM6DSO_GYRO_HIGH_PERFORMANCE_MODE);
	LSM6DSO_GYRO_SetFullScale(&lsm6dso, 500);
	LSM6DSO_GYRO_Set_Filter_Mode(&lsm6dso, 0, LSM6DSO_LP_ODR_DIV_10);
	uint8_t gyro_status = LSM6DSO_GYRO_Enable(&lsm6dso);

	osDelay(10);

	// Accelerometer and gyroscope scale factors
	float acc_scale = 9.81f / 1000.0f;
	float gyro_scale = (3.14159f / 180.0f) / 1000.0f;

	LSM6DSO_Axes_t acc, gyro;

	// Calibrate the IMU
	uint16_t samples = 1000;
	Vector3 acc_offset, gyro_offset;

	for (uint16_t i = 0; i < samples; i++)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		acc_offset += Vector3(acc.x, acc.y, acc.z);
		gyro_offset += Vector3(gyro.x, gyro.y, gyro.z);

		osDelay(2);
	}

	acc_offset *= (acc_scale / (float)samples);
	gyro_offset *= (gyro_scale / (float)samples);

	acc_offset.z -= 9.81f; // Subtract gravity from the z-axis

	statistics_t stats;

	while (true)
	{
		LSM6DSO_ACC_GetAxes(&lsm6dso, &acc);
		LSM6DSO_GYRO_GetAxes(&lsm6dso, &gyro);

		imu_acceleration.x = (acc.x * acc_scale) - acc_offset.x;
		imu_acceleration.y = (acc.y * acc_scale) - acc_offset.y;
		imu_acceleration.z = (acc.z * acc_scale) - acc_offset.z;

		imu_angular_velocity.x = (gyro.x * gyro_scale) - gyro_offset.x;
		imu_angular_velocity.y = (gyro.y * gyro_scale) - gyro_offset.y;
		imu_angular_velocity.z = (gyro.z * gyro_scale) - gyro_offset.z;

		// ComputeStatisticsRecursive(&stats, 1000, imu_angular_velocity.z);

		// Print biases
		// printf("ID: 0x%02X Acceleration: %.4f %.4f %.4f Gyroscope: %.4f %.4f %.4f\n", id, acc_offset.x, acc_offset.y, acc_offset.z, gyro_offset.x, gyro_offset.y, gyro_offset.z);

		// printf("ID: 0x%02X Acceleration: %.2f %.2f %.2f Gyroscope: %.4f %.4f %.4f\n", id, imu_acceleration.x, imu_acceleration.y, imu_acceleration.z, imu_angular_velocity.x, imu_angular_velocity.y, imu_angular_velocity.z);

		osDelay(2);
	}
}

volatile float gps_latitude;
volatile float gps_longitude;
volatile float gps_altitude;

volatile float gps_reference_latitude = 0;
volatile float gps_reference_longitude = 0;
volatile float gps_reference_altitude = 0;

volatile Vector2 gps_velocity;
volatile float gps_orientation_z;

void StartGpsTask(void *argument)
{
	gps.init(&huart4);

	gps.setBaudRate(GPS_BAUDRATE_115200);
	gps.setOutputRate(GPS_ODR_10HZ);

	bool gps_reference = false;

	gps.start();

	osDelay(10);

	Vector2 velocity;

	while (true)
	{
		if (gps.isFixed())
		{
			HAL_GPIO_TogglePin(LED2_STATUS2_PE1_GPIO_Port, LED2_STATUS2_PE1_Pin);
			HAL_GPIO_WritePin(LED3_STATUS3_PE2_GPIO_Port, LED3_STATUS3_PE2_Pin, GPIO_PIN_RESET);

			if (!gps_reference)
			{
				gps_reference_latitude = gps.getLatitude();
				gps_reference_longitude = gps.getLongitude();
				gps_reference_altitude = gps.getAltitude();

				gps_reference = true;
			}

			gps_latitude = gps.getLatitude();
			gps_longitude = gps.getLongitude();
			gps_altitude = gps.getAltitude();

			velocity = gps.getVelocity();
			gps_velocity.x = velocity.x;
			gps_velocity.y = velocity.y;
			gps_orientation_z = gps.getOrientationZ();

			// printf("Latitude: %.8f Longitude: %.8f Altitude: %.2f\n", gps_latitude, gps_longitude, gps_altitude);
			printf("Speed (km/h): %.2f, Velocity: %.2f %.2f Orientation: %.5f\n", velocity.magnitude() * 3.6f, gps_velocity.x, gps_velocity.y, gps_orientation_z);
		}
		else
		{
			HAL_GPIO_WritePin(LED2_STATUS2_PE1_GPIO_Port, LED2_STATUS2_PE1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_STATUS3_PE2_GPIO_Port, LED3_STATUS3_PE2_Pin, GPIO_PIN_SET);

			gps.start(); // Restart the GPS
		}

		osDelay(100);
	}
}

#define TF_LUNA_ADDRESS (0x10 << 1) // Address might be 0x10, shift for STM HAL

volatile float tof_distance;

void StartTofTask(void *argument)
{
	osDelay(10);

	while (true)
	{
		uint8_t data[2] = {0};
		uint8_t status = I2C_Read_Register(&hi2c1, &i2c1MutexHandle, TF_LUNA_ADDRESS, 0x00, data, 2);

		if (status != HAL_OK)
		{
			osDelay(100);
			continue;
		}

		tof_distance = ((data[1] << 8) | data[0]) / 100.0f;

		// printf("Distance: %.2f\n", tof_distance);
		osDelay(50);
	}
}