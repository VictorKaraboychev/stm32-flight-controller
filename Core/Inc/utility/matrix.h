#ifndef __MATRIX_H__
#define __MATRIX_H__

#include <math.h>
#include <stdint.h>
#include <initializer_list>
#include <string.h>
#include <stdio.h>
#include "utility/vector.h"

class Matrix
{
public:
	Matrix();
	Matrix(uint8_t rows, uint8_t cols);
	Matrix(float **data, uint8_t rows, uint8_t cols);
	Matrix(std::initializer_list<std::initializer_list<float>> data);
	Matrix(const Matrix &m);
	~Matrix();

	uint8_t rows() const;
	uint8_t cols() const;

	void set(uint8_t row, uint8_t col, float value);
	float get(uint8_t row, uint8_t col) const;

	Vector *operator[](uint8_t row);
	Vector operator[](uint8_t row) const;

	float &operator()(uint8_t row, uint8_t col);

	Matrix &operator=(const Matrix &m);

	Matrix operator+(const Matrix &m) const;
	Matrix operator-(const Matrix &m) const;
	Matrix operator*(const Matrix &m) const;
	Matrix operator*(const float &s) const;
	Matrix operator/(const float &s) const;

	void operator+=(const Matrix &m);
	void operator-=(const Matrix &m);
	void operator*=(const Matrix &m);
	void operator*=(const float &s);
	void operator/=(const float &s);

	Vector operator*(const Vector &v) const;

	Matrix transpose() const;
	Matrix inverse() const;
	float determinant() const;

	void print() const;

	static Matrix identity(uint8_t size);
	static Matrix fromQuaternion(const float x, const float y, const float z, const float w);

private:
	uint8_t _rows;
	uint8_t _cols;
	float **_data;
};

#endif // __MATRIX_H__