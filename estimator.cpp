#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define STATES 4  // x, xdot, theta, thetadot
#define OBVS 3    // x, theta, thetadot

/*
  This class:
    - stores the constants associated specifically with the beaker2 robot
    - wraps and utilizes the kalman library
*/
class Estimator
{

public:

  Estimator(){

  // Discrete A-Matrix
A << 1,0.02,0.0012156,8.0996e-06,
0,1,0.12171,0.0012156,
0,-4.1282e-08,1.0075,0.02005,
0,-4.1334e-06,0.75343,1.0075;


    // Discrete B-Matrix
  B << 0.00017421,
0.01743,
0.00041282,
0.041334;



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
    R << 0.0001, 0, 0,
         0, 0.001, 0,
         0, 0, 0.001;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.87253,-2.2035,41.208,7.1118;
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

private:

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

  float gain;

  Matrix<1,STATES> K;             // LQR determined K
  Matrix<OBVS,1> y;               // Observed states
  Matrix<STATES,STATES> A;        // State transition
  Matrix<STATES,1> B;             // Force input
  Matrix<OBVS,STATES> C;          // Observing which states
  Matrix<STATES,STATES> Q;        // Process noise covariance
  Matrix<OBVS,OBVS> R;            // Measurement noise covariance
  Matrix<STATES,STATES> P0;       // Initial Estimate error covariance

  KalmanFilter<STATES, OBVS> kf;
};
