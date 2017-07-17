#include <iostream>
#include "../../averager.cpp"

using namespace std;

// for 50 values, with an array size of 16, calculate averages.
int main(){
  Averager a;

  // here are correct answers
  float correctValues[49] = {10.00,10.50,11.00,11.50,12.00,12.50,13.00,13.50,14.00,14.50,15.00,15.50,16.00,16.50,17.00,17.50,18.50,19.50,20.50,21.50,22.50,23.50,24.50,25.50,26.50,27.50,28.50,29.50,30.50,31.50,32.50,33.50,34.50,35.50,36.50,37.50,38.50,39.50,40.50,41.50,42.50,43.50,44.50,45.50,46.50,47.50,48.50,49.50,50.50};


  for(int i = 0; i < 49; i++){

    a.push(i+10);
    if(a.getAvg() != correctValues[i]){
      cout << "error at position: " << i << endl;
    }
  }

  cout << "done!" << endl;
}
