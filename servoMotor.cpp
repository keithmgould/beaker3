#include "Arduino.h"
#include "averager.cpp"
#include <Math.h>

#define WHEEL_RADIUS .042 // in meters
#define FULL_ROTATION_EDGE_EVENTS 300 // 18.75 * 16
#define RADS_PER_SEC_TO_RPM 9.5492965855137
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

class ServoMotor
{
  private:
  Averager averager;
  int edgeCount;
  int lastEdgeCount;
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
    lastEdgeCount = 0;
  }

  // in radians/sec
  float getAvgAngularVelocity() {
    return averager.getAvg();
  }

  // in radians/sec
  float getAngularVelocity() {
    return angularVelocity;
  }

  // in radians/sec
  float getOtherAngularVelocity(long loopTime) {
    // calculate edgeDelta (how many encoder clicks)
    float edgeDelta = (float) abs(edgeCount - lastEdgeCount);
    lastEdgeCount = edgeCount;

    // convert from millis to seconds
    float loopTimeSec = (float) loopTime / 1000;

    //get our radians
    float radians = CLICKS_TO_RADIANS * edgeDelta;

    // return radians per second
    return radians / loopTimeSec;
  }

  // in rotations / min
  float getOtherRPM(float angVel) {
    return angVel * (float) RADS_PER_SEC_TO_RPM;
  }

  // in rotations / min
  float getAvgRPM() {
    return getAvgAngularVelocity() * (float) RADS_PER_SEC_TO_RPM;
  }

  // in rotations / min
  float getRPM() {
    return getAngularVelocity() * (float) RADS_PER_SEC_TO_RPM;
  }

  // in radians
  float getPhi() {
    return (float) edgeCount * (float) CLICKS_TO_RADIANS;
  }

  // in meters
  float getDistance() {
    return getPhi() * (float) WHEEL_RADIUS;
  }

  int getEdgeCount() {
    return edgeCount;
  }

  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};
