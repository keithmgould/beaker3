#include "Arduino.h"
#include <Math.h>

#define PI 3.14159265359
#define WHEEL_DIAMETER .084 // in Meters
#define FULL_ROTATION_EDGE_EVENTS 600 // 18.75 * 32
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

class ServoMotor
{
  private:
  int edgeCount;
  int firstEncoderPin, secondEncoderPin, tickDirection;
  float angularVelocity;
  long secSinceLastMeasure;

  void tickRight()
  {
    edgeCount -= tickDirection;
  }

  void tickLeft()
  {
    edgeCount += tickDirection;
  }

  //-------------------------------------------------------------------
  public:
  //-------------------------------------------------------------------

  ServoMotor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg)
  {
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;
  }

  void init()
  {

    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP); // non-interupt
    secSinceLastMeasure = micros();
    angularVelocity = 0;
    edgeCount = 0;
  }

  // in radians/sec
  void computeAngularVelocity(){
    long secNow = micros();
    float secDelta = (float) (secNow - secSinceLastMeasure) / 1000000;
    angularVelocity = (float) CLICKS_TO_RADIANS / secDelta;
    secSinceLastMeasure = secNow;
  }

  // in radians/sec
  float getAngularVelocity() {
    return angularVelocity;
  }

  // in radians
  float getPhi() {
    return (float) edgeCount * (float) CLICKS_TO_RADIANS;
  }

  // in meters
  float getDistance() {
    return ((float) edgeCount / (float) FULL_ROTATION_EDGE_EVENTS) * 3.14159265 * (float) WHEEL_DIAMETER;
  }

  void encoderEvent() {
    computeAngularVelocity();
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};
