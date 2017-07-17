// Define pins for encoder channels
#define cstEnc0A A8
#define cstEnc0B A9

// Value of Pi
#define pi 3.14159

// Define pins for buttons
#define btnPin1 22  // button 1
#define btnPin4 25  // button 4

// Defnine pins for LED
#define redLED 6
#define greenLED 7
#define blueLED 8

#define nTimeoutFilter 0
#define nFilter 16

// Global variables
volatile word cstPeriodValue4; // Period measured by Timer4
volatile bool cstMicrosec4; // Microsec prescaler used for Timer4
volatile byte portK; // Previous value of port K (A8-A15)

volatile long cstEncVal; // Encoder counter value
volatile word cstEncPeriod; // Encoder period (microsecs)
volatile word avgFilter[nFilter]; // Array for averaging filter
volatile byte iFilter; // Index for averaging filter

volatile byte T4timeout=0; // Timeout to signal stationary encoder


ISR(PCINT2_vect) // Interrupt vector for value change on port K
{
  byte tmpK=PINK; // snapshot of port K
  byte change=portK ^ tmpK; // changed pins
  word T4tmp=TCNT4>>1; // microsec value of timer
  T4timeout=0; // shaft has been moved



  // Updating value of encoder counter
  bool eq0=!(bitRead(tmpK,0)^bitRead(tmpK,1)); // check if channel 0 has changed
  long cstEncValPrev=cstEncVal; // temporary storage for previous value
  cstEncVal=cstEncVal+((bitRead(change,0) & !eq0) | (bitRead(change,1) & eq0)); // incementing encoder
  cstEncVal=cstEncVal-((bitRead(change,0) & eq0) | (bitRead(change,1) & !eq0)); // decrementing encoder


  // Pulse time measurement
  cstEncPeriod=T4tmp; // period in microsect
  avgFilter[iFilter]=T4tmp;
  iFilter=(iFilter<nFilter-1)?(iFilter+1):(0); // incrementing filter array index
  TCNT4=0; // resetting timer

  portK=tmpK; // updating prevous value of port K
}



// Input capture ISR for Timer 4
ISR(TIMER4_CAPT_vect)
{
  // Divide timer value (prescaled by 1:8) by 2 to have direct microsec output
  if (cstMicrosec4)
    cstPeriodValue4=ICR4>>1;
  else
    cstPeriodValue4=ICR4;
  TCNT4H=0; // Reset timer value
  TCNT4L=0;
}

// Overflow ISR for Timer 4
ISR(TIMER4_OVF_vect)
{

  cstPeriodValue4=0; // Set the value of period measurement to 0
  cstEncPeriod=0; // Set the value of period measurement of HW encoder chan 0 to 0
  if (T4timeout>nTimeoutFilter)
  {
    //T4timeout=0; // Set the timeout value to signal stationary shaft
    for (byte k=0;k<nFilter;k++) // Reset values of the averaging filter
      avgFilter[k]=0;
    iFilter=0;
  }
  else
  {
    T4timeout++;
  }
}

// Encoder functions

void setupEncoder() // setting interrupt parameters for encoder
{

  PCICR=PCICR | 1 << 1; // Enable interrupts for A8-A15
  pinMode(cstEnc0A,INPUT); // Set mode of encoder pins
  pinMode(cstEnc0B,INPUT);
  PCICR=PCICR | (1 << 2);
  PCMSK2=PCMSK2 | 0x03; // Set interrupt mask for pins A8 and A9
  TCCR4A=0; // Setting timer configuration registers
  TCCR4B=2;
  TIMSK4=0x01;
  cstEncVal=0; // Resetting encoder value
  cstEncPeriod=0; // Resetting encoder period
}


double getSpeed() // get speed value in rad/sec
{
  if (T4timeout>nTimeoutFilter) // set speed to 0 in case of stationary shaft
    return 0;
  long filteredValue=0; // counting the average of filtered values/4
  for (byte k=0;k<nFilter;k++)
    filteredValue+=avgFilter[k]<<2; //5: nFilter=128
  return 1000000/((filteredValue)/pi);

}

double getEncoderValue() // get direct value of encoder counter
{
  return cstEncVal;
}

double getRadPosition() //  get position in radians
{
  return (cstEncVal/64.0)*pi; // scaling the output
}

void setDutyCycle(int dutyCycle) // set duty cycle of the motor (-255 ... 255)
{
  if (dutyCycle < 0) {
    // set negative direction
      digitalWrite(27,LOW);
      digitalWrite(26,HIGH);
  } else
  if (dutyCycle>0)
  {
    // set positive direction
      digitalWrite(26,LOW);
      digitalWrite(27,HIGH);
  }
  else
  {
    digitalWrite(26,HIGH);
    digitalWrite(27,HIGH);
  }
  // set duty cycle
  analogWrite(4,min(abs(dutyCycle),255));
}

void setVoltage(double voltage) // set voltage of the motor (-5 V ... 5 V)
{
 if (voltage > 0) {
    // set negative direction
    digitalWrite(27,LOW);
    digitalWrite(26,HIGH);
 } else {
    // set positive direction
    digitalWrite(26,LOW);
    digitalWrite(27,HIGH);
  }
  // set duty cycle according to the voltage
  analogWrite(4,min(abs(round(voltage*51)),255));
}


void setupMotor() // set pins connected to the bridge as outputs
{
   pinMode(26,OUTPUT);
   pinMode(27,OUTPUT);
}


void setupButtons() // set I/O channels for the buttons
{
  pinMode(btnPin1,INPUT_PULLUP);
  pinMode(btnPin4,INPUT_PULLUP);
}


void setupLED() // set I/O channels for the LED
{
   pinMode(redLED,OUTPUT);
   pinMode(greenLED,OUTPUT);
   pinMode(blueLED,OUTPUT);

}
