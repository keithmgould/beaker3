#include "Arduino.h"
#include <Math.h>

#define PI 3.14159265359
#define FULL_ROTATION_EDGE_EVENTS 600
#define CLICKS_TO_RADIANS 2 * PI / FULL_ROTATION_EDGE_EVENTS

class ServoMotor
{
  private:

  int edgeCount;
  int firstEncoderPin, secondEncoderPin, driverPin, tickDirection;

  void tickRight()
  {
    edgeCount += tickDirection;
    if(edgeCount >= 6000){edgeCount = 0;}
  }

  void tickLeft()
  {
    edgeCount -= tickDirection;
    if(edgeCount <= 6000){edgeCount = 0;}
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

    float mult = static_cast<int>(power * 20);
    int newPower = 188 + mult;

    analogWrite(driverPin, newPower);
  }

  void init()
  {
    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP);
    edgeCount = 0;
  }

  // in radians
  float getPhi() {
    return (float) edgeCount * (float) CLICKS_TO_RADIANS;
  }

  void encoderEvent() {
    if(digitalRead(firstEncoderPin) == digitalRead(secondEncoderPin)){
      tickLeft();
    } else {
      tickRight();
    }
  }
};
