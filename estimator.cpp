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

  Estimator(int timestep){
    dt = (float) timestep / 1000.0;

    float MassWheels = 0.2,     // kg
          MassRobot = 1.8,      // kg
          Friction = 0.1,       // ?
          Length = 0.209,       // m
          Inertia = 0.06,       // ?
          Gravity = 9.81;       // m/s/s

    // helper variables
    float Denom = Inertia * (MassWheels + MassRobot) + MassWheels * MassRobot * Length * Length;
    float A22 = -(Inertia + MassRobot * Length * Length) * Friction / Denom;
    float A23 = (MassRobot * MassRobot * Gravity * Length * Length) / Denom;
    float A42 = -(MassRobot * Length * Friction) / Denom;
    float A43 = MassRobot * Gravity * Length * (MassWheels + MassRobot) / Denom;
    float B21 = (Inertia + MassRobot * Length * Length) / Denom;
    float B41 = (MassRobot * Length) / Denom;

    // System Matrix
    A << 0, 1, 0, 0,
         0, A22, A23, 0,
         0, 0, 0, 1,
         0, A42, A43, 0;

    // Input Matrix
    B << 0,
         B21,
         0,
         B41;

    // Define the output matrix
    // We only observe x (meters) and theta (radians).
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

    // LQR generated gain
    K << -0.4472, -1.6725, 44.4373, 7.7887;
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
    // y << xPos, theta;
    y << 0, theta;
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
