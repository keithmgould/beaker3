#include <BasicLinearAlgebra.h>
#include <kalman.h>

#define MY_DIM_N 4 // number of states (x, x_dot, theta, theta_dot)
#define MY_DIM_M 2 // number of observations (x, theta)

// NOTE: Soon we can change M to 3 since we can also observe theta_DOT
// via gyrometer

/*
  This class:
    - wraps and utilizes the kalman library
    - stores the constants associated with the model
*/
class Estimator
{

public:

  Estimator(int timestep){
    dt = (float) timestep / 1000.0;

    float MassWheels = 0.2,
          MassRobot = 1.5,
          Friction = 0.1,
          Length = 0.3,
          Inertia = 0.006,
          Gravity = 9.81;

    float  Denom = (Inertia * (MassWheels + MassRobot) + MassWheels * MassRobot * Length * Length);

    // System Matrix
    A << 0, 1, 0, 0,
         0, -(Inertia + MassRobot * Length * Length) * Friction / Denom, MassRobot * MassRobot * Gravity * Length * Length / Denom, 0,
         0, 0, 0, 1,
         0, -MassRobot * Length * Friction / Denom, MassRobot * Gravity * Length * (MassWheels + MassRobot) / Denom, 0;

    // Define the output matrix
    C << 1, 0, 0, 0,
         0, 0, 1, 0;

    // Process noise covariance matrices
    Q << .05, .05, .01, .01,
         .05, .05, .01, .01,
         .01,  .01,  .01, .01,
         .01,  .01,  .01, .01;

    // Measurement noise covariance
    R(0,0) = 5;

    // Estimate error covariance
    P << .1, .1,   .1,    .1,
         .1,  10000,10,    1,
         .1,  10,   100,   10;

  }

  void init(){
    kf.BuildFilter(dt, A, C, Q, R, P);

    // Initial state: zeroed out.
    Matrix<MY_DIM_N, 1> x0;
    x0 << 0, 0, 0, 0;
    Serial << "x0: " <<  x0 << '\n';
    kf.init(0, x0);
  }

  void update(const float xPos, const float theta){
    y << xPos, theta;
    kf.update(y);
  }

  void print(){
    Serial << "y: ";
    Serial << ~y;
    Serial.print(", x_hat");
    Serial.print(": ");
    Serial << ~kf.state() << '\n';
  }

private:
  // double t;
  double dt;

  Matrix<MY_DIM_M, 1> y; // estimated output
  Matrix<MY_DIM_N,MY_DIM_N> A; // System dynamics matrix
  Matrix<MY_DIM_M,MY_DIM_N> C; // Output matrix
  Matrix<MY_DIM_N,MY_DIM_N> Q; // Process noise covariance
  Matrix<MY_DIM_M,MY_DIM_M> R; // Measurement noise covariance
  Matrix<MY_DIM_N,MY_DIM_N> P; // Estimate error covariance
  KalmanFilter<MY_DIM_N, MY_DIM_M> kf;
};
