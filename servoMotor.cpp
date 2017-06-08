#include "Arduino.h"

#define FULL_ROTATION_EDGE_EVENTS 600
#define WHEEL_DIAMETER 84 // in MM

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

  ServoMotor(int firstEncoderPinArg, int secondEncoderPinArg, int tickDirectionArg)
  {
    firstEncoderPin = firstEncoderPinArg;
    secondEncoderPin = secondEncoderPinArg;
    tickDirection = tickDirectionArg;
  }

  void init()
  {
    pinMode(firstEncoderPin, INPUT_PULLUP); // interupt
    pinMode(secondEncoderPin, INPUT_PULLUP);
    edgeCount = 0;
  }

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
