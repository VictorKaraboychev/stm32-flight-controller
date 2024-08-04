#ifndef __GPS_H__
#define __GPS_H__

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usart.h"

enum GPS_BAUDRATE
{
	GPS_BAUDRATE_4800 = 4800,
	GPS_BAUDRATE_9600 = 9600,
	GPS_BAUDRATE_19200 = 19200,
	GPS_BAUDRATE_38400 = 38400,
	GPS_BAUDRATE_57600 = 57600,
	GPS_BAUDRATE_115200 = 115200,
	GPS_BAUDRATE_230400 = 230400,
	GPS_BAUDRATE_460800 = 460800
};

enum GPS_ODR
{
	GPS_ODR_1HZ = 1,
	GPS_ODR_2HZ = 2,
	GPS_ODR_4HZ = 4,
	GPS_ODR_5HZ = 5,
	GPS_ODR_10HZ = 10,
	GPS_ODR_20HZ = 20
};


class GPS
{
public:
	UART_HandleTypeDef *huart;

	GPS();
	~GPS();

	void init(UART_HandleTypeDef *huart);
	void start();
	void update();

	float getLatitude();
	float getLongitude();
	float getAltitude();

	void setBaudRate(GPS_BAUDRATE baudrate);
	void setOutputRate(GPS_ODR odr);

	void UART_Callback();
private:
	// NMEA buffer
	uint8_t rx_byte;
	uint8_t rx_buffer[256];
	uint8_t rx_index;

	// calculated values
	float longitude;
	float latitude;
	float altitude;

	// GGA - Global Positioning System Fixed Data
	float nmea_longitude;
	float nmea_latitude;
	float utc_time;
	char ns, ew;
	int lock;
	int satelites;
	float hdop;
	char msl_units;

	// RMC - Recommended Minimmum Specific GNS Data
	char rmc_status;
	float speed_k;
	float course_d;
	int date;

	// GLL
	char gll_status;

	// VTG - Course over ground, ground speed
	float course_t; // ground speed true
	char course_t_unit;
	float course_m; // magnetic
	char course_m_unit;
	char speed_k_unit;
	float speed_km; // speek km/hr
	char speed_km_unit;

	int validate(const uint8_t *nmea);
	void parse(const uint8_t *nmea);
	float nmea_to_dec(float deg_coord, char nsew);
};

extern GPS gps;

#endif // __GPS_H__
