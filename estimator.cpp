#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define STATES 4
#define OBVS 3
#define PI 3.14159265359

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
    A << 1,0.0061043,0.00046038,3.6851e-06,
         0,0.043089,0.031767,0.00046038,
         0,-0.027391,1.0065,0.020045,
         0,-1.89,0.6231,1.0065;


    // Discrete B-Matrix
    B << 0.017022,
         1.1722,
         0.033554,
         2.3153;


    // observe xpos, theta, thetaDot
    C << 1, 0, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Process noise covariance matrices
    Q << 0.00001, 0, 0, 0,
         0, 0.00001, 0, 0,
         0, 0, 0.00001, 0,
         0, 0, 0, 0.00001;

    // Measurement noise covariance
    R << 0.001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.35943,-1.3537,3.3644,0.72339;
  }

  void init(){
    kf.BuildFilter(A, B, C, Q, R, P0);

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(x0);
  }

  float update(const float xPos, const float theta, const float thetaDot){
    y << xPos, theta, thetaDot;
    kf.update(y, gain);
    gain = (-K * kf.state())(0,0);
    print();
    return gain;
  }

  void printStates(){
    for(int n = 0; n < STATES; n++){
      Serial.print(kf.state()(n,0), 5);
      Serial.print(',');
    }
  }

  void printObservations(){
    for(int n = 0; n < OBVS; n++){
      Serial.print(y(n,0), 5);
      Serial.print(',');
    }
  }

  void print(){
    printObservations();
    printStates();
    Serial.println(gain, 5);
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
