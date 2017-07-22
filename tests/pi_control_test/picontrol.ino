#include <StandardCplusplus.h>
#include <iostream>
#include <string>
#include <iomanip> // setprecision
#include <sstream> // stringstream
#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>
#include "../../servoMotor.cpp"

// indicator light on robot
#define INDICATOR 36

// pins for the motor encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 38
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 40

// We communicate with the sabertooth
// motor driver over serial
#define SerialTX 18

// target speed in rpm
#define SETSPEED_RPM 300
#define P_CONTROL 0.05
#define I_CONTROL 0.03

// 50 Hz
#define TIMESTEP 20

long timeMarker;
long leftOver;

float prevError;
float prevU;
float avg_speed_total;
float last_speed_val;
int value_count;

ServoMotor motorLeft(LH_ENCODER_A,LH_ENCODER_B, -1);
ServoMotor motorRight(RH_ENCODER_A,RH_ENCODER_B, 1);

SoftwareSerial SWSerial(NOT_A_PIN, SerialTX); // RX on no pin (unused). Tx to S1.
SabertoothSimplified sabertooth(SWSerial); // Use SWSerial as the serial port.

void leftEncoderEvent() {
  motorLeft.encoderEvent();
}

void rightEncoderEvent() {
  motorRight.encoderEvent();
}

void updatePower(float newGain){
  // safety first
  if(newGain > 100){newGain = 100;}
  if(newGain < -100){newGain = -100;}

  sabertooth.motor(1, newGain);
  sabertooth.motor(2, newGain);
}

// kill motors and blink status indicator
void errorMode(const char* input) {
  Serial.println(input);
  updatePower(0);
  pinMode(INDICATOR, OUTPUT);
  while(true){
    digitalWrite(INDICATOR, HIGH);
    delay(300);
    digitalWrite(INDICATOR, LOW);
    delay(300);
  }
}

void setup(){
  pinMode(INDICATOR, OUTPUT);
  // turn off indicator light while we setup
  digitalWrite(INDICATOR, LOW);

  // Open serial communications
  // and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;}

  // initialize motor encoders and interrupts
  motorLeft.init();
  motorRight.init();
  delay(100);
  attachInterrupt(digitalPinToInterrupt(LH_ENCODER_A), leftEncoderEvent, RISING);
  attachInterrupt(digitalPinToInterrupt(RH_ENCODER_A), rightEncoderEvent, RISING);

  // Sabertooth motor driver commanded over Serial
  SWSerial.begin(9600);
  while (!SWSerial) {;}
  updatePower(0);

  prevError = 0;
  prevU = 0;
  timeMarker = millis();
  leftOver = 0;
}

void loop(){
  long delta = millis() - timeMarker;
  if(delta < TIMESTEP){leftOver++; return;}
  timeMarker = millis();

  float otherAV = motorLeft.getOtherAngularVelocity(delta);
  float otherRpm = otherAV * 9.5492965855137;

  float error = SETSPEED_RPM - otherRpm;
  float newU = prevU + P_CONTROL * error - I_CONTROL * prevError;
  prevError = error;
  prevU = newU;
  updatePower(newU);

  std::stringstream stm;
  stm << std::fixed;
  stm << std::setprecision(5);
  stm << delta << ",";
  stm << error << ",";
  stm << otherAV << ",";
  stm << otherRpm << ",";
  stm << leftOver;
  std::string str = stm.str();
  Serial.println(str.c_str());
  leftOver = 0;
}
