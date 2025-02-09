#include <frc/estimator/KalmanFilter.h>
#include <frc/system/plant/LinearSystem.h>
#include <frc/system/plant/LinearSystemId.h>
#include <vector>
#include <iostream>

// Constants for the system
double kv = 1.0; // Velocity constant
double ka = 1.0; // Acceleration constant
double dt = 0.02; // Time step in seconds

// Create a linear system representing the system to be controlled
auto system = frc::LinearSystemId::IdentifyVelocitySystem(kv, ka);

// Create Kalman filter
vector<2, 1> stateStdDevs{0.1, 0.1};
frc::Matrix<1, 1> measurementStdDevs{0.1};

frc::KalmanFilter<2, 1, 1> kalmanFilter(
    system,
    stateStdDevs,
    measurementStdDevs,
    dt
);

// Control input (e.g., motor voltage)
frc::Vector<1> u{12.0};

// Measurement (e.g., encoder velocity)
frc::Vector<1> y{3.0};

// Predict the next state
kalmanFilter.Predict(u);

// Correct the predicted state with the measurement
kalmanFilter.Correct(u, y);

// Get the estimated state
frc::Vector<2> xHat = kalmanFilter.Xhat();
