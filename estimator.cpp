#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define MY_DIM_N 4
#define MY_DIM_M 1

/*
  This class:
    - wraps and utilizes the kalman library
    - stores the constants associated with the model
*/
class Estimator
{

public:

  Estimator(){
    dt = 1.0/30; // Time step

    // Discrete LTI projectile motion, measuring position only
    A << 1, dt, 0,
         0, 1, dt,
         0, 0, 1;

    // just observe position
    C << 1, 0, 0;

    // Process noise covariance matrices
    Q << .05, .05, .0,
         .05, .05, .0,
         .0,  .0,  .0;

    // Measurement noise covariance
    R(0,0) = 5;

    // Estimate error covariance
    P << .1, .1, .1,
         .1, 10000, 10,
         .1, 10, 100;

    KalmanFilter<MY_DIM_N, MY_DIM_M> kf (dt, A, C, Q, R, P);

  }

private:
  double dt;



  Matrix<MY_DIM_N,MY_DIM_N> A; // System dynamics matrix
  Matrix<MY_DIM_M,MY_DIM_N> C; // Output matrix
  Matrix<MY_DIM_N,MY_DIM_N> Q; // Process noise covariance
  Matrix<MY_DIM_M,MY_DIM_M> R; // Measurement noise covariance
  Matrix<MY_DIM_N,MY_DIM_N> P; // Estimate error covariance

};
