#include "utility/vector.h"

// Vector class implementation

Vector::Vector()
{
	this->_data = NULL;
	this->_size = 0;
}

Vector::Vector(uint8_t size)
{
	this->_data = new float[size]{0};
	this->_size = size;
}

Vector::Vector(float *data, uint8_t size)
{
	this->_data = new float[size];
	this->_size = size;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] = data[i];
	}
}

Vector::Vector(const Vector &v)
{
	this->_data = new float[v._size];
	this->_size = v._size;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] = v._data[i];
	}
}

Vector::Vector(const Vector3 &v)
{
	this->_data = new float[3];
	this->_size = 3;

	this->_data[0] = v.x;
	this->_data[1] = v.y;
	this->_data[2] = v.z;
}

Vector::Vector(const Vector2 &v)
{
	this->_data = new float[2];
	this->_size = 2;

	this->_data[0] = v.x;
	this->_data[1] = v.y;
}

Vector::~Vector()
{
	if (this->_data != NULL)
	{
		delete[] this->_data;
	}
}

uint8_t Vector::size() const
{
	return this->_size;
}

void Vector::set(uint8_t index, float value)
{
	if (index < this->_size)
	{
		this->_data[index] = value;
	}
}

float Vector::get(uint8_t index) const
{
	return (index < this->_size) ? this->_data[index] : 0;
}

float &Vector::operator[](uint8_t index)
{
	return this->_data[index];
}

float Vector::operator[](uint8_t index) const
{
	return this->_data[index];
}

float &Vector::operator()(uint8_t index)
{
	return this->_data[index];
}

float Vector::operator()(uint8_t index) const
{
	return this->_data[index];
}

Vector Vector::operator-() const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = -this->_data[i];
	}

	return result;
}

Vector Vector::operator+(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] + v._data[i];
	}

	return result;
}

Vector Vector::operator-(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] - v._data[i];
	}

	return result;
}

Vector Vector::operator*(const Vector &v) const
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return Vector();
	}

	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] * v._data[i];
	}

	return result;
}

Vector Vector::operator*(const float &s) const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] * s;
	}

	return result;
}

Vector Vector::operator/(const float &s) const
{
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] / s;
	}

	return result;
}

void Vector::operator+=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] += v._data[i];
	}
}

void Vector::operator-=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] -= v._data[i];
	}
}

void Vector::operator*=(const Vector &v)
{
	if (this->_size != v._size)
	{
		throw "Vector sizes do not match";
		return;
	}

	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] *= v._data[i];
	}
}

void Vector::operator*=(const float &s)
{
	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] *= s;
	}
}

void Vector::operator/=(const float &s)
{
	for (uint8_t i = 0; i < this->_size; i++)
	{
		this->_data[i] /= s;
	}
}

float Vector::magnitude() const
{
	float sum = 0;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		sum += this->_data[i] * this->_data[i];
	}

	return sqrt(sum);
}

Vector Vector::normalize() const
{
	float mag = magnitude();
	Vector result(this->_size);

	for (uint8_t i = 0; i < this->_size; i++)
	{
		result._data[i] = this->_data[i] / mag;
	}

	return result;
}

float Vector::dot(const Vector &v) const
{
	if (this->_size != v._size)
	{
		return 0;
	}

	float sum = 0;

	for (uint8_t i = 0; i < this->_size; i++)
	{
		sum += this->_data[i] * v._data[i];
	}

	return sum;
}

Vector Vector::cross(const Vector &v) const
{
	if (this->_size != 3 || v._size != 3)
	{
		throw "Cross product is only defined for 3D vectors";
		return Vector();
	}

	Vector result(3);

	result._data[0] = this->_data[1] * v._data[2] - this->_data[2] * v._data[1]; // i = j * k - k * j
	result._data[1] = this->_data[2] * v._data[0] - this->_data[0] * v._data[2]; // j = k * i - i * k
	result._data[2] = this->_data[0] * v._data[1] - this->_data[1] * v._data[0]; // k = i * j - j * i

	return result;
}

// Vector3 class implementation

Vector3::Vector3()
{
	x = 0;
	y = 0;
	z = 0;
}

Vector3::Vector3(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

Vector3::Vector3(const Vector3 &v)
{
	this->x = v.x;
	this->y = v.y;
	this->z = v.z;
}

Vector3::~Vector3()
{
}

Vector3 Vector3::operator-()
{
	return Vector3(-x, -y, -z);
}

Vector3 Vector3::operator+(const Vector3 &v) const
{
	return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator-(const Vector3 &v) const
{
	return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator*(const Vector3 &v) const
{
	return Vector3(x * v.x, y * v.y, z * v.z);
}

Vector3 Vector3::operator*(const float &s) const
{
	return Vector3(x * s, y * s, z * s);
}

Vector3 Vector3::operator/(const float &s) const
{
	return Vector3(x / s, y / s, z / s);
}

void Vector3::operator+=(const Vector3 &v)
{
	x += v.x;
	y += v.y;
	z += v.z;
}

void Vector3::operator-=(const Vector3 &v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
}

void Vector3::operator*=(const float &s)
{
	x *= s;
	y *= s;
	z *= s;
}

void Vector3::operator/=(const float &s)
{
	x /= s;
	y /= s;
	z /= s;
}

float Vector3::magnitude() const
{
	return sqrt(x * x + y * y + z * z);
}

Vector3 Vector3::normalize() const
{
	float mag = magnitude();
	return Vector3(x / mag, y / mag, z / mag);
}

Vector3 Vector3::cross(const Vector3 &v)
{
	return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
}

float Vector3::dot(const Vector3 &v)
{
	return x * v.x + y * v.y + z * v.z;
}

// Vector2 class implementation

Vector2::Vector2()
{
	x = 0;
	y = 0;
}

Vector2::Vector2(float x, float y)
{
	this->x = x;
	this->y = y;
}

Vector2::Vector2(const Vector2 &v)
{
	this->x = v.x;
	this->y = v.y;
}

Vector2::~Vector2()
{
}

float Vector2::magnitude()
{
	return sqrt(x * x + y * y);
}

Vector2 Vector2::normalize()
{
	float mag = magnitude();
	return Vector2(x / mag, y / mag);
}

Vector2 Vector2::operator+(const Vector2 &v)
{
	return Vector2(x + v.x, y + v.y);
}

Vector2 Vector2::operator-(const Vector2 &v)
{
	return Vector2(x - v.x, y - v.y);
}

Vector2 Vector2::operator*(const float &s)
{
	return Vector2(x * s, y * s);
}

Vector2 Vector2::operator/(const float &s)
{
	return Vector2(x / s, y / s);
}

void Vector2::operator+=(const Vector2 &v)
{
	x += v.x;
	y += v.y;
}

void Vector2::operator-=(const Vector2 &v)
{
	x -= v.x;
	y -= v.y;
}

void Vector2::operator*=(const float &s)
{
	x *= s;
	y *= s;
}

void Vector2::operator/=(const float &s)
{
	x /= s;
	y /= s;
}

Vector2 Vector2::operator*(const Vector2 &v)
{
	return Vector2(x * v.x, y * v.y);
}

Vector2 Vector2::operator-()
{
	return Vector2(-x, -y);
}

float Vector2::dot(const Vector2 &v)
{
	return x * v.x + y * v.y;
}