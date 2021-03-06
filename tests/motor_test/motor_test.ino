#include <StandardCplusplus.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

// IMU constants
#define BALANCED_OFFSET 0.89 // sensor does not show 0 on balance but it should.

// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 36

#include "../../servoMotor.cpp"

// pins for the motor encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 38
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 40

// We communicate with the sabertooth motor driver
// over serial
#define SerialTX 18

// Time (in millisecs) between loops.
// 20 => 50hz
// 4  => 250hz
#define TIMESTEP 20

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, 1);



SoftwareSerial SWSerial(NOT_A_PIN, SerialTX); // RX on no pin (unused). Tx to S1.
SabertoothSimplified sabertooth(SWSerial); // Use SWSerial as the serial port.

float currentTheta = 0; // can we do float() to declare instead of 0?
long timeMarker = 0;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 40);

void leftEncoderEvent() {
  motorLeft.encoderEvent();
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
  sabertooth.motor(1, newGain);
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
  return (motorLeft.getDistance() + motorLeft.getDistance()) / 2.0;
}

// in radians
float rawPhi() {
  return (motorLeft.getPhi() + motorLeft.getPhi()) / 2.0;
}

// get avg angular velocity
// in rads/sec
float rawPhiDot() {
  return motorLeft.getOtherAngularVelocity(TIMESTEP);
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

  delay(100);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, RISING);


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
  delay(1000);
  updatePower(40);
}

float av = 0;

void loop() {
  long timeDelta = millis() - timeMarker;
  if(timeDelta < TIMESTEP){return;}
  timeMarker = millis();

  av = rawPhiDot();
  Serial.println(av, 3);
}
