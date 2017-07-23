/*
  Either because I am lazy, or because I am not lazy, I wrote my own
  little averager class (that does not use STL). It's disgusting because
  the size of the average is not really dynamic. Its stored in the
  AVERAGE_LENGTH declaration.
*/

#define AVERAGE_LENGTH 16

class Averager
{
  private:

  float array_values[AVERAGE_LENGTH];
  float avg;
  int fillPosition;
  bool stillFilling;

  float sumArray()
  {
    float total = 0;
    for(int i = 0; i < AVERAGE_LENGTH; i++)
    {
      total += array_values[i];
    }

    return  total;
  }

  void calculateAverage(){
    float total = sumArray();

    if(stillFilling){
      avg = total / (float) fillPosition;
    }else{
      avg = total / (float) AVERAGE_LENGTH;
    }
  }

  public:

  Averager()
  {
    fillPosition = 0;
    stillFilling = true;
    avg = 0;
    for(int i = 0; i < AVERAGE_LENGTH; i++)
    {
      array_values[i] = 0;
    }
  }

  void push(float newVal)
  {
    array_values[fillPosition] = newVal;
    if(stillFilling == true && fillPosition == AVERAGE_LENGTH - 1) { stillFilling = false; }
    fillPosition = (fillPosition + 1) % AVERAGE_LENGTH;
  }

  float getAvg()
  {
    calculateAverage();

    return avg;
  }
};
