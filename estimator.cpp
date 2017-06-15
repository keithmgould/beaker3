#include <BasicLinearAlgebra.h>
#include <kalman.h>

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
    dt = 0.05;

    // Discrete A-Matrix
    A << 1.0000, 0.0499, 0.0129, 0.0002,
         0,      0.9948, 0.5218, 0.0129,
         0,     -0.0003, 1.0687, 0.0511,
         0,     -0.0141, 2.7775, 1.0687;

    // Discrete B-Matrix
    B << 0.0013,
         0.0515,
         0.0035,
         0.1414;

    C << 1, 0, 0, 0,
         0, 0, 1, 0;

    // Process noise covariance matrices
    Q << .01, 0, 0,  0,
         0, .01, 0,  0,
         0, 0, .01, 0,
         0, 0, 0, .01;

    // Measurement noise covariance
    R << 0.00001, 0,
         0, 0.01;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.4644, -1.4000, 35.1906, 6.8943;
  }

  void init(){
    kf.BuildFilter(dt, A, B, C, Q, R, P0);

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(0, x0);

    Serial << "y(x), y(th), x^(x), x^(x.), x^(th), x^(th.), gain\n";
  }

  float update(const float xPos, const float theta){
    y << xPos, theta;
    kf.update(y, gain);
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
  double dt;
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
