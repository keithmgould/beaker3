#include <BasicLinearAlgebra.h>
#include <kalman.h>

// NOTE: Soon we can change M to 3 since we can also observe theta_DOT
// via gyrometer

/*
  This class:
    - stores the constants associated specifically with the beaker2 model
    - wraps and utilizes the kalman library
*/

#define NUM_STATES 4
#define NUM_OBVS 2

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
    R << 0.001, 0,
         0, 0.000001;

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
  }

  float update(const float theta, const float phi){
    y << theta, phi;
    kf.update(y, gain);
    gain = (-K * kf.state())(0,0);
    print();
    return gain;
  }

  void printStates(){
    for(int n = 0; n < NUM_STATES; n++){
      Serial << kf.state()(0,0);
      if(n < NUM_STATES - 1){
        Serial << ',';
      }
    }
  }

  void print(){
    Serial << y(0,0) << ',' << y(1,0);
    Serial << kf.state()(0,0) << ',' << kf.state()(1,0) << ',';
    Serial << kf.state()(2,0) << ',' << kf.state()(3,0) << ',';
    Serial << gain << '\n';
  }

private:
  float gain;
  Matrix<NUM_STATES,1> setPoint; // desired state
  Matrix<NUM_STATES,1> setPointDelta; // difference between state and setpoint
  Matrix<1,NUM_STATES> K;  // LQR determined K
  Matrix<NUM_OBVS,1> y; // estimated output
  Matrix<NUM_STATES,NUM_STATES> A;  // System dynamics matrix
  Matrix<NUM_STATES,1> B;  // System dynamics matrix
  Matrix<NUM_OBVS,NUM_STATES> C;  // Output matrix
  Matrix<NUM_STATES,NUM_STATES> Q;  // Process noise covariance
  Matrix<NUM_OBVS,NUM_OBVS> R;  // Measurement noise covariance
  Matrix<NUM_STATES,NUM_STATES> P0; // Initial Estimate error covariance
  KalmanFilter<NUM_STATES, NUM_OBVS> kf;
};
