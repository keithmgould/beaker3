#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define PI 3.14159265359

// NOTE: Soon we can change M to 3 since we can also observe theta_DOT
// via gyrometer

/*
  This class:
    - stores the constants associated specifically with the beaker2 model
    - wraps and utilizes the kalman library
*/
class Estimator
{

public:

  Estimator(){
    // Discrete A-Matrix
    A << 1,0.02,-0.032364,-0.00021564,
         0,1,-3.2406,-0.032364,
         0,0,1.0079,0.020053,
         0,0,0.79595,1.0079;

    // Discrete B-Matrix
    B << 0.087262,
         8.7309,
        -0.0087748,
        -0.87864;


    // only observe theta and phi
    C << 1, 0, 0, 0,
         0, 0, 1, 0;

    // Process noise covariance matrices
    Q << .01, 0, 0,  0,
         0, .01, 0,  0,
         0, 0, .01, 0,
         0, 0, 0, .01;

    // Measurement noise covariance
    R << 0.0000001, 0,
         0, 0.01;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.040218,-0.05903,-6.3242,-1.2316;
  }

  void init(){
    kf.BuildFilter(A, B, C, Q, R, P0);

    // for now setpoint does not change
    setPoint << 0, 0, 0, 0;

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(x0);

    Serial << "A: " << '\n';
    Serial << A << '\n';
    Serial << "y(th), y(phi), x^(th), x^(th_dot), x^(phi), x^(phi_dot), gain\n";
  }

  float update(const float xPos, const float theta){
    y << xPos, theta;
    kf.update(y, gain);
    setPointDelta = kf.state() - setPoint;
    gain = (-K * kf.state())(0,0);
    print();
    return gain;
  }

  void print(){
    Serial << y(0,0) << ',' << y(1,0);
    Serial << ',' << kf.state()(0,0) << ',' << kf.state()(1,0) << ',';
    Serial << kf.state()(2,0) << ',' << kf.state()(3,0) << ',';
    Serial << gain << '\n';
  }

private:
  float gain;
  Matrix<4,1> setPoint; // desired state
  Matrix<4,1> setPointDelta; // difference between state and setpoint
  Matrix<1,4> K;  // LQR determined K
  Matrix<2,1> y; // estimated output
  Matrix<4,4> A;  // System dynamics matrix
  Matrix<4,1> B;  // System dynamics matrix
  Matrix<2,4> C;  // Output matrix
  Matrix<4,4> Q;  // Process noise covariance
  Matrix<2,2> R;  // Measurement noise covariance
  Matrix<4,4> P0; // Initial Estimate error covariance
  KalmanFilter<4, 2> kf;
};
