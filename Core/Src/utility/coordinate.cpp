#include "coordinate.h"

Coordinate::Coordinate()
{
	this->latitude = 0;
	this->longitude = 0;
	this->altitude = 0;
}

Coordinate::Coordinate(float latitude, float longitude, float altitude)
{
	this->latitude = latitude;
	this->longitude = longitude;
	this->altitude = altitude;
}

Coordinate::Coordinate(const Coordinate &c)
{
	this->latitude = c.latitude;
	this->longitude = c.longitude;
	this->altitude = c.altitude;
}

Coordinate::~Coordinate()
{
}

Coordinate &Coordinate::operator=(const Coordinate &c)
{
	this->latitude = c.latitude;
	this->longitude = c.longitude;
	this->altitude = c.altitude;

	return *this;
}

Coordinate Coordinate::operator+(const Coordinate &c) const
{
	return Coordinate(this->latitude + c.latitude, this->longitude + c.longitude, this->altitude + c.altitude);
}

Coordinate Coordinate::operator-(const Coordinate &c) const
{
	return Coordinate(this->latitude - c.latitude, this->longitude - c.longitude, this->altitude - c.altitude);
}

Coordinate Coordinate::operator*(const float &s) const
{
	return Coordinate(this->latitude * s, this->longitude * s, this->altitude * s);
}

Coordinate Coordinate::operator/(const float &s) const
{
	return Coordinate(this->latitude / s, this->longitude / s, this->altitude / s);
}

void Coordinate::operator+=(const Coordinate &c)
{
	this->latitude += c.latitude;
	this->longitude += c.longitude;
	this->altitude += c.altitude;
}

void Coordinate::operator-=(const Coordinate &c)
{
	this->latitude -= c.latitude;
	this->longitude -= c.longitude;
	this->altitude -= c.altitude;
}

void Coordinate::operator*=(const float &s)
{
	this->latitude *= s;
	this->longitude *= s;
	this->altitude *= s;
}

void Coordinate::operator/=(const float &s)
{
	this->latitude /= s;
	this->longitude /= s;
	this->altitude /= s;
}

float Coordinate::distance(const Coordinate &c, float radius) const
{
	float rLat1 = this->latitude * M_PI / 180.0f;
	float rLat2 = c.latitude * M_PI / 180.0f;
	float rLon1 = this->longitude * M_PI / 180.0f;
	float rLon2 = c.longitude * M_PI / 180.0f;

	float dLat = rLat2 - rLat1;
	float dLon = rLon2 - rLon1;

	float a = pow(sin(dLat / 2.0f), 2) + cos(rLat1) * cos(rLat2) * pow(sin(dLon / 2.0f), 2);
	float rads = 2.0f * asin(sqrt(a));

	return rads * radius;
}

float Coordinate::angleTo(const Coordinate &c) const
{
	float rLat1 = this->latitude * M_PI / 180.0f;
	float rLat2 = c.latitude * M_PI / 180.0f;
	float rLon1 = this->longitude * M_PI / 180.0f;
	float rLon2 = c.longitude * M_PI / 180.0f;

	float dLon = rLon2 - rLon1;

	float y = sin(dLon) * cos(rLat2);
	float x = cos(rLat1) * sin(rLat2) - sin(rLat1) * cos(rLat2) * cos(dLon);

	return atan2(y, x);
}

Vector Coordinate::toXYZRectangular(const Coordinate &reference) const
{
	float rLat = this->latitude * M_PI / 180.0f;
	float rLon = this->longitude * M_PI / 180.0f;
	float rAlt = this->altitude;

	float rLat0 = reference.latitude * M_PI / 180.0f;
	float rLon0 = reference.longitude * M_PI / 180.0f;
	float rAlt0 = reference.altitude;

	float x = (rAlt + rAlt0) * cos(rLat) * cos(rLon - rLon0);
	float y = (rAlt + rAlt0) * cos(rLat) * sin(rLon - rLon0);
	float z = (rAlt + rAlt0) * sin(rLat);

	return Vector{x, y, z};
}

void Coordinate::print() const
{
	printf("Lat: %.6f Lng: %.6f Alt: %.2f\n", this->latitude, this->longitude, this->altitude);
}