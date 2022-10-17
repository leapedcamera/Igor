#ifndef IGOR_CONTROL
#define IGOR_CONTROL

#include "PidCascade.h"
   
std::unique_ptr<RobotController> control(new PidCascade);

#endif