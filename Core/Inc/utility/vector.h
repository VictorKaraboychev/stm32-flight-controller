#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <math.h>
#include <stdint.h>

#define VECTOR3_ZERO Vector3(0, 0, 0)
#define VECTOR3_ONE Vector3(1, 1, 1)

class Vector3
{
public:
	Vector3();
	Vector3(float x, float y, float z);
	Vector3(const Vector3 &v);
	~Vector3();

	float x;
	float y;
	float z;

	float &operator[](uint8_t index);
	float operator[](uint8_t index) const;

	Vector3 operator-();

	Vector3 operator+(const Vector3 &v) const;
	Vector3 operator-(const Vector3 &v) const;
	Vector3 operator*(const Vector3 &v) const;
	Vector3 operator*(const float &s) const;
	Vector3 operator/(const float &s) const;

	void operator+=(const Vector3 &v);
	void operator-=(const Vector3 &v);
	void operator*=(const float &s);
	void operator/=(const float &s);

	float magnitude() const;
	Vector3 normalize() const;

	float dot(const Vector3 &v);
	Vector3 cross(const Vector3 &v);
};

#define VECTOR2_ZERO Vector2(0, 0)
#define VECTOR2_ONE Vector2(1, 1)

class Vector2
{
public:
	Vector2();
	Vector2(float x, float y);
	Vector2(const Vector2 &v);
	~Vector2();

	float x;
	float y;

	float magnitude();
	Vector2 normalize();

	Vector2 operator+(const Vector2 &v);
	Vector2 operator-(const Vector2 &v);
	Vector2 operator*(const float &s);
	Vector2 operator/(const float &s);

	void operator+=(const Vector2 &v);
	void operator-=(const Vector2 &v);
	void operator*=(const float &s);
	void operator/=(const float &s);

	Vector2 operator*(const Vector2 &v);
	Vector2 operator-();

	float dot(const Vector2 &v);
};

class Vector
{
public:
	Vector();
	Vector(uint8_t size);
	Vector(float *data, uint8_t size);
	Vector(const Vector &v);
	Vector(const Vector3 &v);
	Vector(const Vector2 &v);
	~Vector();

	// float &x();
	// float &y();
	// float &z();

	uint8_t size() const;

	void set(uint8_t index, float value);
	float get(uint8_t index) const;

	float &operator[](uint8_t index);
	float operator[](uint8_t index) const;

	float &operator()(uint8_t index);
	float operator()(uint8_t index) const;

	Vector operator-() const;

	Vector operator+(const Vector &v) const;
	Vector operator-(const Vector &v) const;
	Vector operator*(const Vector &v) const;
	Vector operator*(const float &s) const;
	Vector operator/(const float &s) const;

	void operator+=(const Vector &v);
	void operator-=(const Vector &v);
	void operator*=(const Vector &v);
	void operator*=(const float &s);
	void operator/=(const float &s);

	float magnitude() const;
	Vector normalize() const;

	float dot(const Vector &v) const;
	Vector cross(const Vector &v) const;

private:
	uint8_t _size;
	float *_data;
};

#endif // __VECTOR_H__