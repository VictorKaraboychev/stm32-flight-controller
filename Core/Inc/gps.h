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
	void stop();

	float getLatitude();
	float getLongitude();
	float getAltitude();

	void getVelocity(float *x, float *y);
	float getOrientationZ();

	bool isFixed();

	void setBaudRate(GPS_BAUDRATE baudrate);
	void setOutputRate(GPS_ODR odr);

	void UART_Callback();
private:
	bool active = false;

	// NMEA buffer
	uint8_t rx_byte;
	uint8_t rx_buffer[256];
	uint8_t rx_index;

	uint32_t baudrate = 0;
	uint64_t lastUpdate;

	// calculated values
	float longitude;
	float latitude;
	float altitude;

	// GGA - Global Positioning System Fixed Data
	float nmea_longitude; // Longitude
	float nmea_latitude; // Latitude
	float utc_time; // UTC time
	char ns, ew; // North/South, East/West
	int lock; // GPS quality indicator
	int satelites; // Number of satelites
	float hdop; // Horizontal dilution of precision

	// RMC - Recommended Minimmum Specific GNS Data
	char rmc_status; // Status
	float speed_nmi; // Speed nmi/h
	float course_d; // Course in degrees
	int date;

	// GLL
	char gll_status;

	// VTG - Course over ground, ground speed
	float course_t; // Ground speed orientation
	float course_m; // Magnetic orientation
	float speed_km; // Speed km/hr

	// TXT
	bool antenna_status; // Antenna status

	void parse(const uint8_t *nmea);
};

extern GPS gps;

#endif // __GPS_H__
