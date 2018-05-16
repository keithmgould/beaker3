// IMU constants
// #define BALANCED_OFFSET 0.895 // sensor does not show 0 on balance but it should.
#define BALANCED_OFFSET 0.90

// pi / 180, for degrees to radians
#define PI_OVER_ONE_EIGHTY 0.017453292519943

// LED on side of robot
#define INDICATOR 8



// pins for the motor encoders
#define RH_ENCODER_A 2 // interupt pin
#define RH_ENCODER_B 4
#define LH_ENCODER_A 3 // interupt pin
#define LH_ENCODER_B 5

// We communicate with the sabertooth motor driver
// over serial
#define SerialTX 18

// Time (in millisecs) between loops.
// 20 => 50hz
#define TIMESTEP 20

#define K1 -0.04743
#define K2 -0.12315
#define K3 -4.932
#define K4 -0.90

// Reinforcement Learning Constants
#define KBESTDIFF 0.10 // what % of best do we take when exploring
#define EPISODE_LENGTH 4000 // how long (ms) does an episode last?

// -0.05031,-0.12736,-4.92791,-0.90338
// -0.04230,-0.13077,-4.95339,-0.90855,62.00000
// -0.05404,-0.12336,-4.95755,-0.91739,155.00000
// -0.04743,-0.12315,-4.93200,-0.90000
