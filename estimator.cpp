#include <StandardCplusplus.h>
#include <iostream>
#include <string>
#include <iomanip> // setprecision
#include <sstream> // stringstream
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
    A << 1,0.02,-0.027862,-0.00018565,
         0,1,-2.7897,-0.027862,
         0,0,1.0076,0.020051,
         0,0,0.76432,1.0076;

    // Discrete B-Matrix
    B << 0.1018,
         10.186,
         -0.012042,
         -1.2058;

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
         0, 0.1, 0,
         0, 0, 0.000001;

    // Initial Estimate error covariance
    P0.Fill(0);

    // DLQR generated gain
    K << -0.008439,-0.014151,-2.0355,-0.37787; // DLQR
    // K << -0.0005,   -0.0017,   -1.0742,   -0.1798; // DLQR less on phi
    // K <<  -0.0002,   -0.0009,   -1.0276,   -0.1695;
    // K << -0.0001,   -0.0005,   -1.0040,   -0.1643;
    // K << -0.0032, -0.0060, -1.3451, -0.2323; // manual

  }

  void init(){
    kf.BuildFilter(A, B, C, Q, R, P0);

    // Initial state: zeroed out.
    Matrix<4, 1> x0;
    x0 << 0, 0, 0, 0;
    kf.init(x0);
  }

  float update(const long loopTime, const float xPos, const float theta, const float thetaDot){
    y << xPos, theta, thetaDot;
    long beforeUpdate = millis();
    kf.update(y, gain);
    long updateTime = millis() - beforeUpdate;
    gain = (-K * kf.state())(0,0);
    print(loopTime, updateTime);
    return gain;
  }

private:

  // Because sending too much over
  // Serial slows things down.
  float boundFloat(float val){
    val = val > 100 ? 100 : val;
    val = val < -100 ? -100 : val;
    return val;
  }

  void printStates(std::stringstream &stm){
    for(int n = 0; n < STATES; n++){
      stm << boundFloat(kf.state()(n,0)) << ",";
    }
  }

  void printObservations(std::stringstream &stm){
    for(int n = 0; n < OBVS; n++){
      stm << boundFloat(y(n,0)) << ",";
    }
  }

  // loopTime, updateTime, o1,o2,o3, s1, s2, s3, s4
  void print(long loopTime, long updateTime){
    std::stringstream stm;
    stm << std::fixed;
    stm << std::setprecision(5);
    stm << loopTime << ",";
    stm << updateTime << ",";
    printObservations(stm);
    printStates(stm);
    stm << std::setprecision(5);
    stm << boundFloat(gain);
    std::string str = stm.str();
    Serial.println(str.c_str());
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
