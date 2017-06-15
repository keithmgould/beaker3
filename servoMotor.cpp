#include "Arduino.h"
#include <Servo.h>
#include <Math.h>

#define FULL_ROTATION_EDGE_EVENTS 600
#define WHEEL_DIAMETER .084 // in Meters

class ServoMotor
{
  private:

  Servo servo;
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

  //===========================================
  // API:
  // full forward: 1
  // full reverse: -1
  // stop: 0

  // convert this to the Sabertooth Servo values:
  // full forward: 180
  // full reverse: 0
  // stop: 90
  void updatePower(float power)
  {
    // safety first.
    if(power > 1) { power = 1; }
    if(power < -1) { power = -1; }

    float mult = power * 90;
    int newPower = 90 + static_cast<int>(mult);

    servo.write(newPower);
  }

  void init()
  {
    servo.attach(driverPin, 1000, 2000);
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
