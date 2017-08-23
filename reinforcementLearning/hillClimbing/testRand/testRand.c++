#include <stdio.h>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <chrono>

using namespace std::chrono;


float RandomFloat(float a, float b) {
  float random = ((float) rand()) / (float) RAND_MAX;
  float diff = b - a;
  float r = random * diff;
  return a + r;
}

int adder=0;

float RandomFloat2() {
  return ((float) rand()) / (float) RAND_MAX;
}


int main(){

/* initialize random seed: */
  srand ( time(NULL) );

  for(int x = 0; x < 100; x++){
    printf("%lf\n", RandomFloat(-.05, .05));
  }

  return 0;
}
