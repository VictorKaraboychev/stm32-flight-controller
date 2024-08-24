#ifndef __EXTENDED_KALMAN_FILTER_H__
#define __EXTENDED_KALMAN_FILTER_H__

#include "utility/vector.h"
#include "utility/matrix.h"

class ExtendedKalmanFilter
{
public:
	ExtendedKalmanFilter();
	ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), Matrix &Q, Matrix &R);
	~ExtendedKalmanFilter();

	void initialize(const Vector &x, const Matrix &P);
	void predict(const Vector &u, float delta_time);
	void update(const Vector &z);

	Vector getState() const;

private:
	Vector (*f)(const Vector &x, const Vector &u); // State transition function
	Matrix (*F)(const Vector &x, const Vector &u); // State transition Jacobian

	Vector (*h)(const Vector &x); // Measurement function
	Matrix (*H)(const Vector &x); // Measurement Jacobian

	Matrix Q; // Process noise covariance
	Matrix R; // Measurement noise covariance

	Vector x; // State estimate
	Matrix P; // Estimate covariance
};

#endif // __EXTENDED_KALMAN_FILTER_H__