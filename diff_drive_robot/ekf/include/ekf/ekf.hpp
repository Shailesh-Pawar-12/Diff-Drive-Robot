#include <iostream>
#include <Eigen/Dense>
#include <memory>
#include <vector>

using namespace std;
using namespace Eigen;

// Base EKF class definition
class EKF {
public:
    EKF(int state_size, int measurement_size);
    
    // Initialize the EKF with initial state and covariance
    virtual void Init(const VectorXd& initial_state, const MatrixXd& initial_covariance);
    
    // Predict the next state using control inputs
    virtual void Predict(float dt, const VectorXd& control_input) = 0;
    
    // Update the state estimate with sensor measurements
    virtual void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) = 0;
    
    // Get the current state estimate
    VectorXd GetState() const { return state; }

    // Covariance matrix
    MatrixXd GetCovariance() const { return covariance; }

protected:
    VectorXd state;             // Robot state
    MatrixXd covariance;        // Covariance matrix
    MatrixXd Q;                 // Process noise covariance
    MatrixXd R;                 // Measurement noise covariance
};

// UnicycleEKF class definition
class UnicycleEKF : public EKF {
public:
    UnicycleEKF();
    void Predict(float dt, const VectorXd& control_input) override;
    void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) override;
};

// DifferentialDriveEKF class definition
class DifferentialDriveEKF : public EKF {
public:
    DifferentialDriveEKF();
    void Predict(float dt, const VectorXd& control_input) override;
    void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) override;
};

// OmniDriveEKF class definition
class OmniDriveEKF : public EKF {
public:
    OmniDriveEKF();
    void Predict(float dt, const VectorXd& control_input) override;
    void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) override;
};

// TricycleEKF class definition (5 state variables: x, y, θ, v, φ)
class TricycleEKF : public EKF {
public:
    TricycleEKF();
    void Predict(float dt, const VectorXd& control_input) override;
    void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) override;

private:
    float L;  // Wheelbase (distance between front and rear wheels)
};


// BicycleEKF class definition (5 state variables: x, y, θ, v, β)
class BicycleEKF : public EKF {
public:
    BicycleEKF();
    void Predict(float dt, const VectorXd& control_input) override;
    void Update(const VectorXd& measurement, const MatrixXd& H, const MatrixXd& R) override;

private:
    float L;  // Wheelbase (distance between front and rear wheels)
};

