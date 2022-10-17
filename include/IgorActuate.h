#ifndef IGOR_ACTUATE
#define IGOR_ACTUATE

#include <MotorController.h>
#include "Wheel.h"

// Actuator
double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
MotorController motorController(S_MOTOR_A, F_MOTOR_A, R_MOTOR_A,
 S_MOTOR_B, F_MOTOR_B, R_MOTOR_B, motorSpeedFactorLeft, motorSpeedFactorRight);

// Wheels
Wheel wheelLeft(W_ENCODER_A, Y_ENCODER_A);
Wheel wheelRight(W_ENCODER_B, Y_ENCODER_A);


#endif