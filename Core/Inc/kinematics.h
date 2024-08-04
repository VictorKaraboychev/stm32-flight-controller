#ifndef __KINEMATICS_H__
#define __KINEMATICS_H__

#include "main.h"
#include "utility/vector.h"
#include "utility/matrix.h"

#define KINEMATICS_UPDATE_FREQUENCY 100

#define ACCELEROMETER_VARIANCE 0.01f // 0.01 m/s^2
#define GYROSCOPE_VARIANCE 0.001f    // 0.001 rad/s
#define MAGNETOMETER_VARIANCE 0.05f  // 0.05 rad
#define GPS_VARIANCE 2.0f            // 2.0 m
#define BAROMETER_VARIANCE 1.0f      // 1.0 m

class Kinematics
{
public:
	Kinematics();
	~Kinematics();

	void begin();
	void update();
	void reset();

	float timeSinceLastUpdate();

	Vector3 getPosition();
	Vector3 getVelocity();
	Vector3 getAcceleration();

	Vector3 getOrientation();
	Vector3 getAngularVelocity();
	Vector3 getAngularAcceleration();
private:
	Vector3 position;
	Vector3 velocity;
	Vector3 acceleration;

	Vector3 orientation;
	Vector3 angularVelocity;
	Vector3 angularAcceleration;

	uint64_t lastUpdate;
};

extern Kinematics kinematics;

void kinematicsTask(void *pvParameters);

#endif // __KINEMATICS_H__