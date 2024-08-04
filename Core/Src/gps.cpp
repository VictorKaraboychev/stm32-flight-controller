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

void setBaudrate(UART_HandleTypeDef *huart, uint32_t baudrate)
{
	HAL_UART_DeInit(huart);
	huart->Init.BaudRate = baudrate;
	HAL_UART_Init(huart);
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
}

void GPS::start()
{
	HAL_UART_Receive_IT(this->huart, &rx_byte, 1);
}

void GPS::UART_Callback()
{
	if (rx_byte == '\n')
	{
		if (validate(rx_buffer))
		{
			printf("%s\n", rx_buffer);
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

	// Send command to GPS in all baudrates
	setBaudrate(this->huart, 4800);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 9600);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 19200);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 38400);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 57600);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 115200);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 230400);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);
	setBaudrate(this->huart, 460800);
	HAL_UART_Transmit(this->huart, nmea, sizeof(nmea), 100);

	setBaudrate(this->huart, baudrate);
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

int GPS::validate(const uint8_t *nmea)
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

void GPS::parse(const uint8_t *nmea)
{
	char *c = (char *)nmea;

	if (!strncmp(c, "$GPGGA", 6))
	{
		if (sscanf(c, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &utc_time, &nmea_latitude, &ns, &nmea_longitude, &ew, &lock, &satelites, &hdop, &msl_altitude, &msl_units) >= 1)
		{
			latitude = nmea_to_dec(nmea_latitude, ns);
			longitude = nmea_to_dec(nmea_longitude, ew);
			return;
		}
	}
	else if (!strncmp(c, "$GPRMC", 6))
	{
		if (sscanf(c, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &utc_time, &nmea_latitude, &ns, &nmea_longitude, &ew, &speed_k, &course_d, &date) >= 1)
			return;
	}
	else if (!strncmp(c, "$GPGLL", 6))
	{
		if (sscanf(c, "$GPGLL,%f,%c,%f,%c,%f,%c", &nmea_latitude, &ns, &nmea_longitude, &ew, &utc_time, &gll_status) >= 1)
			return;
	}
	else if (!strncmp(c, "$GPVTG", 6))
	{
		if (sscanf(c, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &course_t, &course_t_unit, &course_m, &course_m_unit, &speed_k, &speed_k_unit, &speed_km, &speed_km_unit) >= 1)
			return;
	}

}

float GPS::nmea_to_dec(float deg_coord, char nsew)
{
	int degree = (int)(deg_coord / 100);
	float minutes = deg_coord - degree * 100;
	float dec_deg = minutes / 60;
	float decimal = degree + dec_deg;
	if (nsew == 'S' || nsew == 'W')
	{ // return negative
		decimal *= -1;
	}
	return decimal;
}

// int GPS_validate(uint8_t *nmeastr)
// {
//     char check[3];
//     char checkcalcstr[3];
//     int i;
//     int calculated_check;

//     i = 0;
//     calculated_check = 0;

//     // check to ensure that the string starts with a $
//     if (nmeastr[i] == '$')
//     {
//         i++;
//     }
//     else
//     {
//         printf("No $ found\n");
//         return 0;
//     }

//     // No NULL reached, 75 char largest possible NMEA message, no '*' reached
//     while ((nmeastr[i] != 0) && (nmeastr[i] != '*') && (i < 75))
//     {
//         calculated_check ^= nmeastr[i]; // calculate the checksum
//         i++;
//     }

//     if (i >= 75)
//     {
//         printf("String too long\n");
//         return 0; // the string was too long so return an error
//     }

//     if (nmeastr[i] == '*')
//     {
//         check[0] = nmeastr[i + 1]; // put hex chars in check string
//         check[1] = nmeastr[i + 2];
//         check[2] = 0;
//     }
//     else
//     {
//         printf("No checksum separator found\n");
//         return 0; // no checksum separator found there for invalid
//     }

//     sprintf(checkcalcstr, "%02X", calculated_check);
//     return ((checkcalcstr[0] == check[0]) && (checkcalcstr[1] == check[1])) ? 1 : 0;
// }

// void GPS_parse(uint8_t *GPSstrParse)
// {
//     if (!strncmp(GPSstrParse, "$GPGGA", 6))
//     {
//         if (sscanf(GPSstrParse, "$GPGGA,%f,%f,%c,%f,%c,%d,%d,%f,%f,%c", &gps.utc_time, &gps.nmea_latitude, &gps.ns, &gps.nmea_longitude, &gps.ew, &gps.lock, &gps.satelites, &gps.hdop, &gps.msl_altitude, &gps.msl_units) >= 1)
//         {
//             gps.dec_latitude = GPS_nmea_to_dec(gps.nmea_latitude, gps.ns);
//             gps.dec_longitude = GPS_nmea_to_dec(gps.nmea_longitude, gps.ew);
//             return;
//         }
//     }
//     else if (!strncmp(GPSstrParse, "$GPRMC", 6))
//     {
//         if (sscanf(GPSstrParse, "$GPRMC,%f,%f,%c,%f,%c,%f,%f,%d", &gps.utc_time, &gps.nmea_latitude, &gps.ns, &gps.nmea_longitude, &gps.ew, &gps.speed_k, &gps.course_d, &gps.date) >= 1)
//             return;
//     }
//     else if (!strncmp(GPSstrParse, "$GPGLL", 6))
//     {
//         if (sscanf(GPSstrParse, "$GPGLL,%f,%c,%f,%c,%f,%c", &gps.nmea_latitude, &gps.ns, &gps.nmea_longitude, &gps.ew, &gps.utc_time, &gps.gll_status) >= 1)
//             return;
//     }
//     else if (!strncmp(GPSstrParse, "$GPVTG", 6))
//     {
//         if (sscanf(GPSstrParse, "$GPVTG,%f,%c,%f,%c,%f,%c,%f,%c", &gps.course_t, &gps.course_t_unit, &gps.course_m, &gps.course_m_unit, &gps.speed_k, &gps.speed_k_unit, &gps.speed_km, &gps.speed_km_unit) >= 1)
//             return;
//     }
// }

// float GPS_nmea_to_dec(float deg_coord, char nsew)
// {
//     int degree = (int)(deg_coord / 100);
//     float minutes = deg_coord - degree * 100;
//     float dec_deg = minutes / 60;
//     float decimal = degree + dec_deg;
//     if (nsew == 'S' || nsew == 'W')
//     { // return negative
//         decimal *= -1;
//     }
//     return decimal;
// }
