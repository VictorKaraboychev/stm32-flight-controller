#include "gps.h"

uint8_t nmeaChecksum(const uint8_t *nmea)
{
	uint8_t checksum = 0;
	uint8_t i = 1; // Skip the '$' character

	while (nmea[i] != '*' && nmea[i] != 0 && i < 75) // Stop at the '*' character
	{
		checksum ^= nmea[i++];
	}

	return checksum;
}

float nmeaToDec(float deg_coord, char nsew)
{
	int degree = (int)(deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;

	// return negative if south or west
	if (nsew == 'S' || nsew == 'W')
	{
		decimal *= -1;
	}
	return decimal;
}

bool validate(const uint8_t *nmea)
{
	// check to ensure that the string starts with a $
	if (nmea[0] != '$')
	{
		printf("No $ found\n");
		return 0;
	}

	uint8_t checksum = 0;
	uint8_t i = 1; // Skip the '$' character

	while (nmea[i] != '*' && nmea[i] != 0 && i < 75) // Stop at the '*' character
	{
		checksum ^= nmea[i++];
	}

	if (nmea[i] == '*') // Check if the checksum is present
	{
		uint8_t check[3] = {nmea[i + 1], nmea[i + 2], 0};
		char checkcalcstr[3];
		sprintf(checkcalcstr, "%02X", checksum);
		return (checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1]);
	}
}

GPS gps;

GPS::GPS()
{
}

GPS::~GPS()
{
}

void GPS::init(UART_HandleTypeDef *huart)
{
	this->huart = huart;

	uint8_t cmdbuf[] = "$PCAS10,0*1C\r\n";

	this->setBaudRate((GPS_BAUDRATE)this->huart->Init.BaudRate);
	HAL_UART_Transmit(this->huart, cmdbuf, sizeof(cmdbuf), 100);
}

void GPS::start()
{
	HAL_UART_Receive_IT(this->huart, &rx_byte, 1);
	this->active = true;
}

void GPS::stop()
{
	this->active = false;
}

void GPS::UART_Callback()
{
	if (!this->active)
	{
		return;
	}

	if (rx_byte == '\n')
	{
		if (validate(rx_buffer))
		{
			// printf("%s\n", rx_buffer);
			parse(rx_buffer);
		}
		rx_index = 0;
		memset(rx_buffer, 0, sizeof(rx_buffer));
	}
	else
	{
		rx_buffer[rx_index++] = rx_byte;
	}

	HAL_UART_Receive_IT(this->huart, &rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == gps.huart->Instance)
	{
		gps.UART_Callback();
	}
}

float GPS::getLatitude()
{
	return latitude;
}

float GPS::getLongitude()
{
	return longitude;
}

float GPS::getAltitude()
{
	return altitude;
}

Vector2 GPS::getVelocity()
{
	float theta = this->getOrientationZ();
	float speed = this->speed_km / 3.6f;

	return VECTOR2_ROTATION(theta) * speed;
}

float GPS::getOrientationZ()
{
	float theta = (M_PI / 180.0f) * -this->course_t + M_PI_2;

	if (theta < 0)
	{
		theta += 2.0f * M_PI;
	}

	return theta;
}

bool GPS::isFixed()
{
	return this->lock && this->antenna_status;
}

void GPS::setBaudRate(GPS_BAUDRATE baudrate)
{
	uint8_t nmea[16] = "$PCAS01,0*";

	switch (baudrate)
	{
	case GPS_BAUDRATE_4800:
		nmea[8] = '0';
		break;
	case GPS_BAUDRATE_9600:
		nmea[8] = '1';
		break;
	case GPS_BAUDRATE_19200:
		nmea[8] = '2';
		break;
	case GPS_BAUDRATE_38400:
		nmea[8] = '3';
		break;
	case GPS_BAUDRATE_57600:
		nmea[8] = '4';
		break;
	case GPS_BAUDRATE_115200:
		nmea[8] = '5';
		break;
	case GPS_BAUDRATE_230400:
		nmea[8] = '6';
		break;
	case GPS_BAUDRATE_460800:
		nmea[8] = '7';
		break;
	default:
		return;
	}

	uint8_t checksum = nmeaChecksum(nmea);

	sprintf((char *)nmea, "%s%02X\r\n", nmea, checksum);

	if (!this->baudrate)
	{
		uint32_t all_baudrates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800};

		// Send command to GPS in all baudrates
		for (uint32_t baud : all_baudrates)
		{
			UART_SetBaudrate(this->huart, baud);
			HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
		}
	}
	else
	{
		UART_SetBaudrate(this->huart, this->baudrate);
		HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	}

	UART_SetBaudrate(this->huart, baudrate);
	this->baudrate = baudrate;
}

void GPS::setOutputRate(GPS_ODR odr)
{
	uint8_t nmea[32];

	uint32_t ms = 1000 / odr;
	sprintf((char *)nmea, "$PCAS02,%lu*", ms);

	uint8_t checksum = nmeaChecksum(nmea);

	sprintf((char *)nmea, "%s%02X\r\n", nmea, checksum);

	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
}

void GPS::parse(const uint8_t *nmea)
{
	char *c = (char *)nmea + 3;

	if (!strncmp(c, "GGA", 3))
	{
		this->lock = 0;
		if (sscanf(c, "GGA,%f,%f,%c,%f,%c,%d,%d,%f,%f", &this->utc_time, &this->nmea_latitude, &this->ns, &this->nmea_longitude, &this->ew, &this->lock, &this->satelites, &this->hdop, &this->altitude) >= 1)
		{
			latitude = nmeaToDec(nmea_latitude, ns);
			longitude = nmeaToDec(nmea_longitude, ew);
			return;
		}
	}
	else if (!strncmp(c, "RMC", 3))
	{
		if (sscanf(c, "RMC,%f,%f,%c,%f,%c,%f,%f,%d", &this->utc_time, &this->nmea_latitude, &this->ns, &this->nmea_longitude, &this->ew, &this->speed_nmi, &this->course_d, &this->date) >= 1)
			return;
	}
	else if (!strncmp(c, "GLL", 3))
	{
		if (sscanf(c, "GLL,%f,%c,%f,%c,%f,%c", &this->nmea_latitude, &this->ns, &this->nmea_longitude, &this->ew, &this->utc_time, &this->gll_status) >= 1)
			return;
	}
	else if (!strncmp(c, "VTG", 3))
	{
		if (sscanf(c, "VTG,%f,T,,M,%f,N,%f,K", &this->course_t, &this->speed_nmi, &this->speed_km) >= 1)
			return;
	}
	else if (!strncmp(c, "TXT", 3))
	{
		char antenna_str[16];
		if (sscanf(c, "TXT,01,01,01,ANTENNA %s*", antenna_str) >= 1)
		{
			this->antenna_status = strncmp(antenna_str, "OK", 2) == 0;
			return;
		}
	}
}