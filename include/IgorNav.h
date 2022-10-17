#ifndef IGOR_NAV
#define IGOR_NAV

#include <NexgenAHRS.h>
#include "State.h"

// Nav
State state;
LSM9DS1 imu;
EulerAngles angles;
SensorData rates;

// Nav-params
float nKp = 9;
float nKi = (float)9/1000;
unsigned long dt;


#endif