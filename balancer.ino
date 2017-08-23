#include <StandardCplusplus.h>
#include <iostream>
#include <string>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <cmath>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

#include "servoMotor.cpp"
#include "constants.cpp"

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, -1);

SoftwareSerial SWSerial(NOT_A_PIN, SerialTX); // RX on no pin (unused). Tx to S1.
SabertoothSimplified sabertooth(SWSerial); // Use SWSerial as the serial port.

float currentTheta = 0; // can we do float() to declare instead of 0?
float gyroAngle = 0;
float dt = 0;
float filteredTheta = 0;
float newPhi = 0;
float newTheta = 0;
float newThetaDot = 0;
float phiDot = 0;
float lastPhi = 0;
long timeDelta = 0;
long timeMarker = 0;
long nowish = 0;
long lastGainReversal = 0;
float oldGain = 0;
float bandStopFilterGain = 0;
long sinceLastDirectionSwitch = 0;
float newGain = 0;
float multiplier = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

void leftEncoderEvent() {
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  motorRight.encoderEvent();
}

// "acceleration" to radians
// sin(theta) = y / 9.8.
// solve for theta
float accToRadians(float acc) {
  float theta = (acc + BALANCED_OFFSET) / 9.8;
  if(theta < -1) { theta = -1; }
  if(theta > 1) { theta = 1; }
  return asin(theta);
}

float degToRadians(float deg) {
  return deg * PI_OVER_ONE_EIGHTY;
}

void updatePower(float newGain){
  // newGain = constrain(newGain * -150, -40, 40);
  newGain = constrain(newGain * -40, -40, 40);

  sabertooth.motor(1, newGain);
  sabertooth.motor(2, newGain);
}

void turnIndicatorLightOff(){
  digitalWrite(INDICATOR, LOW);
}

void turnIndicatorLightOn(){
  digitalWrite(INDICATOR, HIGH);
}

// kill motors and blink status indicator
void errorMode(const char* input) {
  Serial.println(input);
  updatePower(0);
  pinMode(INDICATOR, OUTPUT);
  while(true){
    turnIndicatorLightOn();
    delay(300);
    turnIndicatorLightOff();
    delay(300);
  }
}

// in meters
float rawXPos() {
  return (motorLeft.getDistance() + motorRight.getDistance()) / 2.0;
}

// in radians
float rawPhi() {
  return (motorLeft.getPhi() + motorRight.getPhi()) / 2.0;
}

// in radians
float rawTheta() {
  // Linear Acceleration in meters per second squared
  imu::Vector<3> Acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  return accToRadians(-Acceleration.z());
}

// radians per second
float rawThetaDot(){
  // Angular velocity in degrees per second (needs to be converted)
  imu::Vector<3> RotationalVelocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  return degToRadians(RotationalVelocity.y());
}

float calculateGain(float phi, float phiDot, float theta, float thetaDot){
  return (phi*K1 + phiDot*K2 + theta*K3 + thetaDot*K4);
}

void setup() {
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  turnIndicatorLightOff();

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;}

  // initialize motor encoders and interrupts
  motorLeft.init();
  motorRight.init();
  delay(100);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, CHANGE);

  // Sabertooth motor driver commanded over Serial
  SWSerial.begin(9600);
  while (!SWSerial) {;}
  updatePower(0);

  //Check to see if the Inertial Sensor is functioning
  if (!bno.begin(0x05)) {
    Serial.println("Inertial Sensor failed, or not present");
    errorMode("could not find IMU.");
  } else {
    bno.setExtCrystalUse(true);
  }

  // Turn on indicator light because we are ready to rock.
  turnIndicatorLightOn();

  // now that light is on, allow human to get the robot upright
  delay(2000);

  // print out two lines in case there  was garbage (noise) in serial setup
  Serial.println();
  Serial.println();
  delay(100);

  // ensure our timeDelta is accurate by resetting timeMarker
  // just before the loop starts
  timeMarker = millis();
}

float gain = 0;
int loopCounter = 0;
float seconds = 0;

void printStuff(float seconds, long loopTime, float phi, float phiDot, float theta, float thetaDot, float gain, float multiplier){
  std::stringstream stm;
  stm << std::fixed;
  stm << std::setprecision(5);
  stm << seconds << ",";
  stm << phi << ",";
  stm << phiDot << ",";
  stm << theta << ",";
  stm << thetaDot << ",";
  stm << gain << ",";
  stm << multiplier;
  std::string str = stm.str();
  Serial.println(str.c_str());
}

void loop() {
  // following four lines ensure loop is TIMESTEP ms long
  nowish = millis();
  timeDelta = nowish - timeMarker;
  if(timeDelta < TIMESTEP){return;}
  timeMarker = nowish;
  seconds += timeDelta;

  // fetch values from sensors
  newThetaDot = rawThetaDot();
  newPhi = rawPhi();
  newTheta = rawTheta();

  // if it falls outside its prob noise
  if(newTheta > -0.3 || newTheta < 0.3) {
    dt = (float) timeDelta / 1000;
    filteredTheta = (filteredTheta + newThetaDot * dt) * 0.98 + newTheta * 0.02;
  }

  // calculate phi dot (discrete derivative)
  newPhi = rawPhi();
  phiDot = (newPhi - lastPhi) / dt;
  lastPhi = newPhi;

  oldGain = gain;

  // calculate LQR Gain
  gain = calculateGain(newPhi, phiDot, filteredTheta, newThetaDot);


  //--------------------------------------------------
  // Ghetto Band Stop Filter

  // if gain switched directions
  if ((oldGain > 0 && gain < 0) || (oldGain < 0 && gain > 0)){
    lastGainReversal = millis();
  }

  // note square root of (g/l) ~~ 136ms
  multiplier = fabs(136 - (millis() - lastGainReversal)) * 0.00036765;
  multiplier = constrain(multiplier, 0, 0.8) + 0.2;
  //-------------------------------------------------

  updatePower(gain * multiplier);

  printStuff(seconds, timeDelta, newPhi, phiDot, filteredTheta, newThetaDot, gain, multiplier);
}
