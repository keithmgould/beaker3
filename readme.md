## Relevant Files:

0. maple.m - the matlab file that generates A, B, K matrices
0. balancer.ino - the main arduino file to compile (and upload to robot)
0. estimator.cpp - the full state estimator (wraps Kalman filter)
0. servoMotor.cpp - small class to help read data from encoders

## Dependencies:

0. Wire.h - this is a core Arduino lib
0. Adafruit_Sensor.h - this is a core Ardruino lib
0. Adafruit_BNO055.h - https://github.com/adafruit/Adafruit_BNO055
0. SoftwareSerial.h - this is a core Arduino lib
0. SabertoothSimplified.h - https://www.dimensionengineering.com/info/arduino
0. BasicLinearAlgebra.h - https://github.com/keithmgould/BasicLinearAlgebra
0. kalman.h - https://github.com/keithmgould/kalman-cpp

## STEPS:

0. Run matlab script (maple.m) to generate A, B, K matrices.
0. Copy matrix values into the estimator.cpp file.
0. make the main file (balancer.ino)
