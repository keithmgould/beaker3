## Relevant Files:

0. maple.m - the matlab file that generates A, B, K matrices
0. balancer.ino - the main arduino file to compile (and upload to robot)
0. estimator.cpp - the full state estimator (wraps Kalman filter)
0. servoMotor.cpp - small class to help read data from encoders

## STEPS:

0. Run matlab script (maple.m) to generate A, B, K matrices.
0. Copy matrix values into the estimator.cpp file.
0. make the main file (balancer.ino)
