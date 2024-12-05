#include "ekf/ekf.hpp"

EKF::EKF(int state_size, int measurement_size) {
    state = VectorXd(state_size);
    covariance = MatrixXd::Identity(state_size, state_size); // Initialize covariance to identity
    Q = MatrixXd::Identity(state_size, state_size); // Process noise covariance
    R = MatrixXd::Identity(measurement_size, measurement_size); // Measurement noise covariance
}

void EKF::Init(const VectorXd& initial_state, const MatrixXd& initial_covariance) {
    state = initial_state;
    covariance = initial_covariance;
}

UnicycleEKF::UnicycleEKF() : EKF(4, 2) { // 4 state variables, 2 measurement variables
    // Initialize process noise covariance
    Q << 0.01, 0, 0, 0,
         0, 0.01, 0, 0,
         0, 0, 0.001, 0,
         0, 0, 0, 0.001;
}

void UnicycleEKF::Predict(float dt, const VectorXd& control_input) {
    float v = control_input(0);    // Linear velocity
    float omega = control_input(1); // Angular velocity

    float x = state(0);
    float y = state(1);
    float theta = state(2);

    // Unicycle motion model (basic kinematics)
    state(0) = x + v * cos(theta) * dt;
    state(1) = y + v * sin(theta) * dt;
    state(2) = theta + omega * dt;
    
    // Jacobian of the motion model (A matrix)
    MatrixXd A(4, 4);
    A << 1, 0, -v * sin(theta) * dt, 0,
         0, 1, v * cos(theta) * dt, 0,
         0, 0, 1, dt,
         0, 0, 0, 1;
    
    // Update covariance
    covariance = A * covariance * A.transpose() + Q;
}

void UnicycleEKF::Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + R).inverse();
    
    VectorXd z_hat = H * state;       // Predicted measurement
    VectorXd y = measurement - z_hat; // Measurement residual

    state = state + K * y;           // Update state estimate
    covariance = (MatrixXd::Identity(4, 4) - K * H) * covariance; // Update covariance
}

DifferentialDriveEKF::DifferentialDriveEKF() : EKF(5, 2) { // 5 state variables, 2 measurement variables

    Q << 1, 0, 0, 0, 0,
         0, 1, 0, 0, 0,
         0, 0, 1, 0, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;

}

void DifferentialDriveEKF::Predict(float dt, const VectorXd& control_input) {
    float v = control_input(0);     // Forward velocity
    float omega = control_input(1); // Angular velocity
    
    float x = state(0);
    float y = state(1);
    float theta = state(2);

    // Differential drive model
    state(0) = x + v * cos(theta) * dt;
    state(1) = y + v * sin(theta) * dt;
    state(2) = theta + omega * dt;

    // Jacobian A matrix for differential drive
    MatrixXd A(5, 5);
    A << 1, 0, -v * sin(theta) * dt, 0, 0,
         0, 1, v * cos(theta) * dt, 0, 0,
         0, 0, 1, dt, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;

    // Update covariance
    covariance = A * covariance * A.transpose() + Q;
}

void DifferentialDriveEKF::Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + R).inverse();
    
    VectorXd z_hat = H * state;       // Predicted measurement
    VectorXd y = measurement - z_hat; // Measurement residual

    state = state + K * y;           // Update state estimate
    covariance = (MatrixXd::Identity(5, 5) - K * H) * covariance; // Update covariance
}

OmniDriveEKF::OmniDriveEKF() : EKF(6, 2) { // 6 state variables, 2 measurement variables
    Q << 0.01, 0, 0, 0, 0, 0,
         0, 0.01, 0, 0, 0, 0,
         0, 0, 0.001, 0, 0, 0,
         0, 0, 0, 0.001, 0, 0,
         0, 0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0, 0.001;
}

void OmniDriveEKF::Predict(float dt, const VectorXd& control_input) {
    float v_x = control_input(0);    // x velocity
    float v_y = control_input(1);    // y velocity

    float x = state(0);
    float y = state(1);

    // Omnidirectional model
    state(0) = x + v_x * dt;
    state(1) = y + v_y * dt;

    // Jacobian A matrix for omnidirectional model
    MatrixXd A(6, 6);
    A << 1, 0, 0, dt, 0, 0,
         0, 1, 0, 0, dt, 0,
         0, 0, 1, 0, 0, dt,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;

    // Update covariance
    covariance = A * covariance * A.transpose() + Q;
}

void OmniDriveEKF::Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + R).inverse();
    
    VectorXd z_hat = H * state;       // Predicted measurement
    VectorXd y = measurement - z_hat; // Measurement residual

    state = state + K * y;           // Update state estimate
    covariance = (MatrixXd::Identity(6, 6) - K * H) * covariance; // Update covariance
}

// TricycleEKF (5 state variables, 2 measurement variables)
TricycleEKF::TricycleEKF() : EKF(5, 2), L(1.0f) { // Initialize L with a default value, e.g., 1.0m
    // Process noise covariance Q for Tricycle EKF
    Q << 0.01, 0, 0, 0, 0,
         0, 0.01, 0, 0, 0,
         0, 0, 0.001, 0, 0,
         0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0.001;
}

void TricycleEKF::Predict(float dt, const VectorXd& control_input) {
    float v = control_input(0);    // Linear velocity
    float phi = control_input(1);  // Steering angle

    float x = state(0);
    float y = state(1);
    float theta = state(2);

    // Tricycle motion model with a wheelbase 'L'
    state(0) = x + v * cos(theta) * dt;                        // Update x position
    state(1) = y + v * sin(theta) * dt;                        // Update y position
    state(2) = theta + (v / L) * tan(phi) * dt;                // Update orientation (theta)
    
    // Jacobian matrix (A) for the tricycle model
    MatrixXd A(5, 5);
    A << 1, 0, -v * sin(theta) * dt, 0, 0,
         0, 1, v * cos(theta) * dt, 0, 0,
         0, 0, 1, dt * tan(phi) / L, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;
    
    // Update covariance using process noise covariance Q
    covariance = A * covariance * A.transpose() + Q;
}

void TricycleEKF::Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + R).inverse();
    
    VectorXd z_hat = H * state;       // Predicted measurement
    VectorXd y = measurement - z_hat; // Measurement residual

    state = state + K * y;           // Update state estimate
    covariance = (MatrixXd::Identity(5, 5) - K * H) * covariance; // Update covariance
}

// BicycleEKF (5 state variables, 2 measurement variables)
BicycleEKF::BicycleEKF() : EKF(5, 2), L(1.0f) { // Initialize L with a default value, e.g., 1.0m
    // Process noise covariance Q for Bicycle EKF
    Q << 0.01, 0, 0, 0, 0,
         0, 0.01, 0, 0, 0,
         0, 0, 0.001, 0, 0,
         0, 0, 0, 0.001, 0,
         0, 0, 0, 0, 0.001;
}

void BicycleEKF::Predict(float dt, const VectorXd& control_input) {
    float v = control_input(0);       // Linear velocity
    float delta = control_input(1);   // Steering angle
    float beta = atan(L * tan(delta) / L); // Slip angle

    float x = state(0);
    float y = state(1);
    float theta = state(2);

    // Bicycle motion model with a wheelbase 'L'
    state(0) = x + v * cos(theta + beta) * dt;   // Update x position
    state(1) = y + v * sin(theta + beta) * dt;   // Update y position
    state(2) = theta + (v / L) * sin(beta) * dt; // Update orientation (theta)

    // Jacobian matrix (A) for the bicycle model
    MatrixXd A(5, 5);
    A << 1, 0, -v * sin(theta + beta) * dt, 0, 0,
         0, 1, v * cos(theta + beta) * dt, 0, 0,
         0, 0, 1, dt * sin(beta) / L, 0,
         0, 0, 0, 1, 0,
         0, 0, 0, 0, 1;
    
    // Update covariance using process noise covariance Q
    covariance = A * covariance * A.transpose() + Q;
}


void BicycleEKF::Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) {
    MatrixXd K = covariance * H.transpose() * (H * covariance * H.transpose() + R).inverse();
    
    VectorXd z_hat = H * state;       // Predicted measurement
    VectorXd y = measurement - z_hat; // Measurement residual

    state = state + K * y;           // Update state estimate
    covariance = (MatrixXd::Identity(5, 5) - K * H) * covariance; // Update covariance
}

