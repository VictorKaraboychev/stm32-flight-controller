#ifndef __KALMAN_H__
#define __KALMAN_H__

#include "utility/vector.h"
#include "utility/matrix.h"

class Kalman
{
public:
	Kalman();
	Kalman(const Matrix &A, const Matrix &B, const Matrix &H, const Matrix &Q, const Matrix &R);
	~Kalman();

	void update(const Vector &u, const Vector &z);

	void setState(const Vector &x);
	Vector getState() const;

private:
	Matrix A;
	Matrix B;
	Matrix H;
	Matrix Q;
	Matrix R;

	Vector x;
	Matrix P;
};

#endif // __KALMAN_H__