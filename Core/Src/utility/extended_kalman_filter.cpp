#include "extended_kalman_filter.h"

ExtendedKalmanFilter::ExtendedKalmanFilter(Vector (*f)(const Vector &x, const Vector &u), Matrix (*F)(const Vector &x, const Vector &u), Vector (*h)(const Vector &x), Matrix (*H)(const Vector &x), Matrix &Q, Matrix &R)
{
	this->f = f;
	this->F = F;

	this->h = h;
	this->H = H;

	this->Q = Q;
	this->R = R;

	this->x = Vector(Q.rows());
	this->P = Matrix::identity(Q.rows());
}

ExtendedKalmanFilter::~ExtendedKalmanFilter()
{
}

void ExtendedKalmanFilter::initialize(const Vector &x, const Matrix &P)
{
	this->x = x;
	this->P = P;
}

void ExtendedKalmanFilter::predict(const Vector &u, float delta_time)
{
	Vector f = this->f(this->x, u); // State transition function
	Matrix F = this->F(this->x, u); // Jacobian of state transition function

	// Predict the state estimate
	this->x = this->x + f * delta_time;
	this->P = this->P + (F * this->P + this->P * F.transpose() + this->Q) * delta_time;
}

void ExtendedKalmanFilter::update(const Vector &z)
{
	Vector h = this->h(this->x); // Measurement function
	Matrix H = this->H(this->x); // Jacobian of measurement function

	// Calculate the Kalman gain
	Matrix K = this->P * H.transpose() * (H * this->P * H.transpose() + this->R).inverse();

	// Update the state estimate
	this->x = this->x + K * (z - h);
	this->P = (Matrix::identity(this->P.rows()) - K * H) * this->P;
}

Vector ExtendedKalmanFilter::getState() const
{
	return this->x;
}