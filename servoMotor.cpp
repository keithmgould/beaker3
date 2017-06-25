#include "Arduino.h"
#include <Math.h>

#define FULL_ROTATION_EDGE_EVENTS 600
#define WHEEL_DIAMETER .084 // in Meters

class ServoMotor
{
  private:

  int edgeCount;
  int firstEncoderPin, secondEncoderPin, driverPin, tickDirection;

  void tickRight()
  {
    edgeCount += tickDirection;
  }

  void tickLeft()
  {
    edgeCount -= tickDirection;
  }

  //-------------------------------------------------------------------
  public:
  //-------------------------------------------------------------------

  ServoMotor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg, int driverPinArg)
  {
    driverPin = driverPinArg;
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;
  }

  // static void balancedPower(ServoMotor& servoLeft, ServoMotor& servoRight, float gain){
  //   float leftGain = gain * (servoRight.getDistance() / servoLeft.getDistance());
  //   float rightGain = gain * (servoLeft.getDistance() / servoRight.getDistance());
  //   servoLeft.updatePower(leftGain);
  //   servoRight.updatePower(rightGain);
  // }

  //===========================================
  // API:
  // full forward: 1
  // full reverse: -1
  // stop: 0

  // Convert to PWM.
  // 188 is full stop
  // 208 is full forward
  // 168 is full reverse
  void updatePower(float power)
  {
    // safety first.
    if(power > 1) { power = 1; }
    if(power < -1) { power = -1; }

    float mult = static_cast<int>(power * 40);
    int newPower = 188 + mult;

    analogWrite(driverPin, newPower);
  }

  void init()
  {
    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP);
    edgeCount = 0;
  }

  // in meters
  float getDistance() {
    return ((float) edgeCount / (float) FULL_ROTATION_EDGE_EVENTS) * 3.14159265 * (float) WHEEL_DIAMETER;
  }

  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};
