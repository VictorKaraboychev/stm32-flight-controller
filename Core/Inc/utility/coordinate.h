#ifndef __COORDINATE_H__
#define __COORDINATE_H__

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "vector.h"

class Coordinate
{
public:
	Coordinate();
	Coordinate(float latitude, float longitude, float altitude);
	Coordinate(const Coordinate &c);
	~Coordinate();

	float latitude;
	float longitude;
	float altitude;

	Coordinate &operator=(const Coordinate &c);

	Coordinate operator+(const Coordinate &c) const;
	Coordinate operator-(const Coordinate &c) const;
	Coordinate operator*(const float &s) const;
	Coordinate operator/(const float &s) const;

	void operator+=(const Coordinate &c);
	void operator-=(const Coordinate &c);
	void operator*=(const float &s);
	void operator/=(const float &s);

	float distance(const Coordinate &c, float radius) const; // Haversine formula
	float angleTo(const Coordinate &c) const;

	Vector toXYZRectangular(const Coordinate &reference) const;

	void print() const;
};

#endif /* __COORDINATE_H__ */
