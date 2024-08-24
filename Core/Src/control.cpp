#include "control.h"

void StartStatusLedTask(void *argument)
{
	while (true)
	{
		// Print all sensor statuses
		printf("IMU: %s, BAR: %s, MAG: %s, GPS: %s, TOF: %s\n",
			   imu_data.active ? "Y" : "N",
			   bar_data.active ? "Y" : "N",
			   mag_data.active ? "Y" : "N",
			   gps_data.active ? "Y" : "N",
			   tof_data.active ? "Y" : "N");

		HAL_GPIO_TogglePin(LED1_STATUS1_PE0_GPIO_Port, LED1_STATUS1_PE0_Pin);
		osDelay(500);
	}
}

Vector f(const Vector &x, const Vector &u)
{
	float p = u[0];
	float q = u[1];
	float r = u[2];

	float sin0 = sin(x[0]); // sin(φ)
	float cos0 = cos(x[0]); // cos(φ)
	float cos1 = cos(x[1]); // cos(θ)
	float tan1 = tan(x[1]); // tan(θ)

	// State transition function (3x1 vector)
	return Vector{
		p + (q * sin0 + r * cos0) * tan1, // φ
		q * cos0 - r * sin0,			  // θ
		(q * sin0 + r * cos0) / cos1	  // ψ
	};
}

Matrix F(const Vector &x, const Vector &u)
{
	float p = u[0];
	float q = u[1];
	float r = u[2];

	float sin0 = sin(x[0]); // sin(φ)
	float cos0 = cos(x[0]); // cos(φ)
	float cos1 = cos(x[1]); // cos(θ)
	float tan1 = tan(x[1]); // tan(θ)

	// Jacobian of state transition function (3x3 matrix)
	return Matrix{
		{tan1 * (q * cos0 - r * sin0), (q * sin0 + r * cos0) * (tan1 * tan1 + 1.0f), 0},
		{-q * sin0 - r * cos0, 0, 0},
		{(q * cos0 - r * sin0) / cos1, (q * sin0 + r * cos0) * (tan1 / cos1), 0}};
}

Vector h(const Vector &x)
{
	float sin0 = sin(x[0]); // sin(φ)
	float cos0 = cos(x[0]); // cos(φ)
	float sin1 = sin(x[1]); // sin(θ)
	float cos1 = cos(x[1]); // cos(θ)

	// Measurement function (3x1 vector)
	return Vector{
		GRAVITY * sin1,			// ay
		-GRAVITY * sin0 * cos1, // ax
		-GRAVITY * cos0 * cos1	// az
	};
}

Matrix H(const Vector &x)
{
	float sin0 = sin(x[0]); // sin(φ)
	float cos0 = cos(x[0]); // cos(φ)
	float sin1 = sin(x[1]); // sin(θ)
	float cos1 = cos(x[1]); // cos(θ)

	// Jacobian of measurement function (3x3)
	return Matrix{
		{0, GRAVITY * cos1, 0},
		{-GRAVITY * cos0 * cos1, GRAVITY * sin0 * sin1, 0},
		{GRAVITY * sin0 * cos1, GRAVITY * cos0 * sin1, 0}};
}

void StartFusionTask(void *argument)
{
	Matrix Q = Matrix::identity(3) * 0.001; // Process noise covariance
	Matrix R = Matrix::identity(3) * 0.004; // Measurement noise covariance

	// Create the Extended Kalman Filter
	ExtendedKalmanFilter ekf(f, F, h, H, Q, R);

	// Set the initial state (roll, pitch)
	ekf.initialize(Vector{0, 0, 0}, Matrix::identity(3) * 0.1);

	// Wait for sensors to initialize before starting the filter
	while (!imu_data.active)
	{
		osDelay(100);
	}

	uint64_t last_time = HAL_GetTick();

	while (true)
	{
		// Compute the time delta
		float delta_time = (HAL_GetTick() - last_time) / 1000.0f;
		last_time = HAL_GetTick();

		// Get the IMU data
		Vector u = imu_data.angular_velocity; // p, q, r
		Vector z = imu_data.acceleration;	  // ax, ay, az

		// Predict the state
		ekf.predict(u, delta_time);

		// Update the state
		ekf.update(z);

		// Get the state estimate
		Vector x = ekf.getState();

		// Print the state estimate
		printf("Roll: %.4f Pitch: %.4f Yaw: %.4f\n", x[0], x[1], x[2]);

		osDelay(10);
	}
}

void StartControlTask(void *argument)
{
	while (true)
	{
		osDelay(500);
	}
}