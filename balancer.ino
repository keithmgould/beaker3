#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <BasicLinearAlgebra.h>
#include <kalman.h>

// IMU constants
#define BALANCED_OFFSET 0.85 // sensor does not show 0 on balance but it should.

// 90 degrees in rads is 1.5708
// Given proportion acc/9.8 = y/1.5708 solve for y
// yields wheel multiplier
#define WHEEL_MULTIPLIER 0.1603

// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 36

// Motor Encoder constants
// (define these before including servoMotor.cpp)
#define FULL_ROTATION_EDGE_EVENTS 600
#define WHEEL_DIAMETER 84 // in MM

#include "servoMotor.cpp"

// pins for the motor encoder inputs
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 38
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 40

#define MY_DIM_N 4
#define MY_DIM_M 1

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

void leftEncoderEvent() {
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  motorRight.encoderEvent();
}

float accToRadians(float acc) {
  return WHEEL_MULTIPLIER * (acc - BALANCED_OFFSET);
}

float degToRadians(float deg) {
  return deg * PI_OVER_ONE_EIGHTY;
}

void setup() {
  // Open serial communications
  // and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {}

  motorLeft.init();
  motorRight.init();
  delay(100);

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  pinMode(INDICATOR, OUTPUT);
  digitalWrite(INDICATOR, HIGH);

  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  if (!bno.begin(0x05)) {
    Serial.println("Inertial Sensor failed, or not present");
  } else {

    bno.setExtCrystalUse(true);

    Serial.println("Inertial Sensor present");
  }
}

void loop() {
  imu::Vector<3> Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  //Acceleration in meters per second squared
  imu::Vector<3> RotationalVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE); //Angular velocity in radians per second

  Serial << motorLeft.getDistance() << ", " << motorRight.getDistance() << ", ";
  Serial << accToRadians(Acceleration.z()) << ", " << degToRadians(-RotationalVelocity.y()) << '\n';
  delay(50);
}









