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
    // Discrete A-Matrix
    A <<  1,0.049872,0.01291,0.00021429,
          0,0.99485,0.52181,0.01291,
          0,-0.00034982,1.0687,0.05114,
          0,-0.014139,2.7775,1.0687;

    // Discrete B-Matrix
    B << 0.0012819,
          0.051532,
          0.0034982,
          0.14139;

    C << 1, 0, 0, 0,
         0, 0, 1, 0;

    // Process noise covariance matrices
    Q << .01, 0, 0,  0,
         0, .01, 0,  0,
         0, 0, .01, 0,
         0, 0, 0, .01;

    // Measurement noise covariance
    R << 0.0000001, 0,
         0, 0.00004520629858;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.06834,-0.56392,34.419,4.8054;
  }

  void init(){
    kf.BuildFilter(A, B, C, Q, R, P0);

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(x0);

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
