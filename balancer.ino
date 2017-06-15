#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

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
#include "estimator.cpp"

// pins for the motor power and encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 38
#define RH_POWER 44
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 40
#define LH_POWER 46

#define TIMESTEP 50 // out of 1000 (ms)

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1, LH_POWER);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1, RH_POWER);
Estimator estimator;

float currentTheta = 0; // can we do float() to declare instead of 0?
float newGain = 0;

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

void reportError() {
  pinMode(INDICATOR, OUTPUT);
  while(true){
    digitalWrite(INDICATOR, HIGH);
    delay(300);
    digitalWrite(INDICATOR, LOW);
    delay(300);
  }
}

void updatePower(float newGain){
  if(newGain > 20){newGain = 20;}
  if(newGain < -20){newGain = -20;}

  // notice we are only going to 50% of motor power.
  // 20/40
  float newPower = newGain / 60;

  motorRight.updatePower(newPower);
  motorLeft.updatePower(newPower);
}

float avgXPos() {
  return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
}

void setup() {
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  digitalWrite(INDICATOR, LOW);

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {}

  motorLeft.init();
  motorRight.init();
  delay(100);

  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  //Check to see if the Inertial Sensor is wired correctly and functioning normally
  if (!bno.begin(0x05)) {
    Serial.println("Inertial Sensor failed, or not present");
    reportError();
  } else {

    bno.setExtCrystalUse(true);

    Serial.println("Inertial Sensor present");
  }

  estimator.init();

  // Turn on indicator light because we are ready to rock.
  digitalWrite(INDICATOR, HIGH);
}

void loop() {
  // Acceleration in meters per second squared
  imu::Vector<3> Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Angular velocity in radians per second
  // when ready here is gyro: degToRadians(-RotationalVelocity.y()
  imu::Vector<3> RotationalVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  currentTheta = accToRadians(Acceleration.z());
  newGain = estimator.update(avgXPos(), currentTheta);
  updatePower(newGain);
  delay(TIMESTEP);
}









