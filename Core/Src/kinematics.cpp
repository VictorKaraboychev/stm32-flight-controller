#include "kinematics.h"
#include "kalman.h"
// #include "imu.h"
// #include "gps.h"
// #include "barometer.h"

Kalman kalman;

Kinematics::Kinematics()
{
    position = VECTOR3_ZERO;
    velocity = VECTOR3_ZERO;
    acceleration = VECTOR3_ZERO;

    orientation = VECTOR3_ZERO;
    angularVelocity = VECTOR3_ZERO;
    angularAcceleration = VECTOR3_ZERO;

    lastUpdate = HAL_GetTick();
}

Kinematics::~Kinematics()
{
}

void Kinematics::begin()
{
    float delta_t = 1.0 / KINEMATICS_UPDATE_FREQUENCY;

    // Define state transition matrix (A)
    // Assuming a simplified linear motion model
    Matrix A = Matrix::identity(18);
    A(0, 3) = delta_t;   // x' = x + vx * dt
    A(1, 4) = delta_t;   // y' = y + vy * dt
    A(2, 5) = delta_t;   // z' = z + vz * dt
    A(3, 6) = delta_t;   // vx' = vx + ax * dt
    A(4, 7) = delta_t;   // vy' = vy + ay * dt
    A(5, 8) = delta_t;   // vz' = vz + az * dt
    A(6, 6) = 1.0;       // ax' = ax
    A(7, 7) = 1.0;       // ay' = ay
    A(8, 8) = 1.0;       // az' = az
    A(9, 12) = delta_t;  // phi' = phi + vphi * dt
    A(10, 13) = delta_t; // theta' = theta + vtheta * dt
    A(11, 14) = delta_t; // psi' = psi + vpsi * dt
    A(12, 15) = delta_t; // vphi' = vphi + aphi * dt
    A(13, 16) = delta_t; // vtheta' = vtheta + atheta * dt
    A(14, 17) = delta_t; // vpsi' = vpsi + apsi * dt
    A(15, 15) = 1.0;     // aphi' = aphi
    A(16, 16) = 1.0;     // atheta' = atheta
    A(17, 17) = 1.0;     // apsi' = apsi

    // Define control input matrix (B)
    Matrix B(18, 6);
    B(6, 0) = delta_t;  // External ax affects ax
    B(7, 1) = delta_t;  // External ay affects ay
    B(8, 2) = delta_t;  // External az affects az
    B(15, 3) = delta_t; // External p affects aphi
    B(16, 4) = delta_t; // External q affects atheta
    B(17, 5) = delta_t; // External r affects apsi

    // Define measurement matrix (H)
    Matrix H(9, 18);
    H(0, 0) = 1.0;  // Measure x
    H(1, 1) = 1.0;  // Measure y
    H(2, 2) = 1.0;  // Measure z
    H(3, 9) = 1.0;  // Measure phi
    H(4, 10) = 1.0; // Measure theta
    H(5, 11) = 1.0; // Measure psi
    H(6, 6) = 1.0;  // Measure ax
    H(7, 7) = 1.0;  // Measure ay
    H(8, 8) = 1.0;  // Measure az

    // Define process noise covariance matrix (Q)
    Matrix Q = Matrix::identity(18);
    Q(6, 6) = ACCELEROMETER_VARIANCE * delta_t; // Accelerometer noise variance in x direction
    Q(7, 7) = ACCELEROMETER_VARIANCE * delta_t; // Accelerometer noise variance in y direction
    Q(8, 8) = ACCELEROMETER_VARIANCE * delta_t; // Accelerometer noise variance in z direction
    Q(15, 15) = GYROSCOPE_VARIANCE * delta_t;   // Gyroscope noise variance around x axis (roll)
    Q(16, 16) = GYROSCOPE_VARIANCE * delta_t;   // Gyroscope noise variance around y axis (pitch)
    Q(17, 17) = GYROSCOPE_VARIANCE * delta_t;   // Gyroscope noise variance around z axis (yaw)

    // Define measurement noise covariance matrix (R)
    Matrix R(9, 9);
    R(0, 0) = GPS_VARIANCE;           // GPS x noise variance
    R(1, 1) = GPS_VARIANCE;           // GPS y noise variance
    R(2, 2) = BAROMETER_VARIANCE;     // Barometer z noise variance
    R(3, 3) = MAGNETOMETER_VARIANCE;  // Magnetometer phi noise variance
    R(4, 4) = MAGNETOMETER_VARIANCE;  // Magnetometer theta noise variance
    R(5, 5) = MAGNETOMETER_VARIANCE;  // Magnetometer psi noise variance
    R(6, 6) = ACCELEROMETER_VARIANCE; // Accelerometer ax noise variance
    R(7, 7) = ACCELEROMETER_VARIANCE; // Accelerometer ay noise variance
    R(8, 8) = ACCELEROMETER_VARIANCE; // Accelerometer az noise variance

    // Create the Kalman filter
    kalman = Kalman(A, B, H, Q, R);

    // Initialize the state vector (assuming no motion)
    Vector x0(18);

    // Initialize the Kalman filter
    kalman.update(Vector(6), x0);
}

void Kinematics::update()
{
    uint64_t time = HAL_GetTick();
    float dt = (time - lastUpdate) / 1.0e3;
    lastUpdate = time;

    // Get IMU data
    // Vector3 acceleration = imu.getAcceleration();
    // Vector3 angularVelocity = imu.getAngularVelocity();
    // Vector3 orientation = gps.magneticOrientationToGeoOrientation(imu.getOrientation()); // Correct for magnetic north offset

    // Get GPS data
    // Vector3 position = gps.getPosition();

    // Get barometer data
    // float altitude = barometer.getAltitude();

    // Create the control input vector
    Vector u(6);
    // u(0) = acceleration.x;
    // u(1) = acceleration.y;
    // u(2) = acceleration.z;
    u(3) = angularVelocity.x;
    u(4) = angularVelocity.y;
    u(5) = angularVelocity.z;

    // Create the measurement vector
    Vector z(9);
    // z(0) = position.x;
    // z(1) = position.y;
    // z(2) = altitude;
    // z(3) = orientation.x;
    // z(4) = orientation.y;
    // z(5) = orientation.z;
    // z(6) = acceleration.x;
    // z(7) = acceleration.y;
    // z(8) = acceleration.z;

    // Update the Kalman filter
    kalman.update(u, z);

    // Get the state vector
    Vector x = kalman.getState();

    // Update the position, velocity, and acceleration
    position.x = x(0);
    position.y = x(1);
    position.z = x(2);

    velocity.x = x(3);
    velocity.y = x(4);
    velocity.z = x(5);

    acceleration.x = x(6);
    acceleration.y = x(7);
    acceleration.z = x(8);

    // Update the orientation, angular velocity, and angular acceleration
    orientation.x = x(9);
    orientation.y = x(10);
    orientation.z = x(11);

    angularVelocity.x = x(12);
    angularVelocity.y = x(13);
    angularVelocity.z = x(14);

    angularAcceleration.x = x(15);
    angularAcceleration.y = x(16);
    angularAcceleration.z = x(17);
}

float Kinematics::timeSinceLastUpdate()
{
    return (HAL_GetTick() - lastUpdate) / 1.0e3;
}

Vector3 Kinematics::getPosition()
{
    return position;
}

Vector3 Kinematics::getVelocity()
{
    return velocity;
}

Vector3 Kinematics::getAcceleration()
{
    return acceleration;
}

Vector3 Kinematics::getOrientation()
{
    return orientation;
}

Vector3 Kinematics::getAngularVelocity()
{
    return angularVelocity;
}

Vector3 Kinematics::getAngularAcceleration()
{
    return angularAcceleration;
}

void Kinematics::reset()
{
    position = VECTOR3_ZERO;
    velocity = VECTOR3_ZERO;
    acceleration = VECTOR3_ZERO;

    orientation = VECTOR3_ZERO;
    angularVelocity = VECTOR3_ZERO;
    angularAcceleration = VECTOR3_ZERO;

    // imu.reset();
    // gps.reset();
    // barometer.reset();

    lastUpdate = HAL_GetTick();
}

Kinematics kinematics;

// void kinematicsTask(void *pvParameters)
// {
//     kinematics.begin();

//     while (true)
//     {
//         kinematics.update();

//         osDelay(1000 / KINEMATICS_UPDATE_FREQUENCY);
//     }
// }