#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define STATES 4
#define OBVS 2
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
    A << 1,0.0056717,0.00057558,4.6533e-06,
         0,0.03229,0.038972,0.00057558,
         0,-0.034276,1.0082,0.020056,
         0,-2.3208,0.77295,1.0082;

    // Discrete B-Matrix
    B << 0.017552,
         1.1854,
         0.041988,
         2.843;



    // only observe theta and phi
    C << 1, 0, 0, 0,
         0, 0, 1, 0;

    // Process noise covariance matrices
    Q << .001, 0, 0,  0,
         0, .001, 0,  0,
         0, 0, .01,  0,
         0, 0, 0, .01;

    // Measurement noise covariance
    R << 0.01, 0,
         0, 0.5;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.034266,-0.89682,1.1501,0.39275;
  }

  void init(){
    kf.BuildFilter(A, B, C, Q, R, P0);

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(x0);

    Serial << "y(th), y(phi), x^(th), x^(th_dot), x^(phi), x^(phi_dot), gain\n";
  }

  float update(const float xPos, const float theta){
    y << xPos, theta;
    kf.update(y, gain);
    gain = (-K * kf.state())(0,0);
    // print();
    return gain;
  }

  void printStates(){
    for(int n = 0; n < STATES; n++){
      Serial << kf.state()(n,0) << ',';
    }
  }

  void print(){
    Serial << y(0,0) << ',' << y(1,0) << ',';
    printStates();
    Serial << gain << '\n';
  }

private:
  float gain;
  Matrix<1,STATES> K;  // LQR determined K
  Matrix<OBVS,1> y; // estimated output
  Matrix<STATES,STATES> A;  // System dynamics matrix
  Matrix<STATES,1> B;  // System dynamics matrix
  Matrix<OBVS,STATES> C;  // Output matrix
  Matrix<STATES,STATES> Q;  // Process noise covariance
  Matrix<OBVS,OBVS> R;  // Measurement noise covariance
  Matrix<STATES,STATES> P0; // Initial Estimate error covariance
  KalmanFilter<STATES, OBVS> kf;
};
