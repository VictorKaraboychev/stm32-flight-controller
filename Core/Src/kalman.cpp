#include "kalman.h"

Kalman::Kalman()
{
	this->A = Matrix();
	this->B = Matrix();
	this->H = Matrix();
	this->Q = Matrix();
	this->R = Matrix();

	this->x = Vector();
	this->P = Matrix();
}

Kalman::Kalman(const Matrix &A, const Matrix &B, const Matrix &H, const Matrix &Q, const Matrix &R)
{
	this->A = A;
	this->B = B;
	this->H = H;
	this->Q = Q;
	this->R = R;

	this->x = Vector(A.rows());
	this->P = Matrix(A.rows(), A.rows());
}

Kalman::~Kalman()
{
}

void Kalman::update(const Vector &u, const Vector &z)
{
	// Predict
	this->x = this->A * this->x + this->B * u;
	this->P = this->A * this->P * this->A.transpose() + this->Q;

	// Update
	Matrix S = this->H * this->P * this->H.transpose() + this->R;
	Matrix K = this->P * this->H.transpose() * S.inverse();

	this->x = this->x + K * (z - this->H * this->x);
	this->P = (Matrix::identity(this->P.rows()) - K * this->H) * this->P;
}

void Kalman::setState(const Vector &x)
{
	this->x = x;
}

Vector Kalman::getState() const
{
	return this->x;
}