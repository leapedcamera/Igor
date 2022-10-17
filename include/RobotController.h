#ifndef RobotController_h
#define RobotController_h
#include "Arduino.h"
#include "State.h"

class RobotController
{

public:
    RobotController(){};
    virtual void initControl() = 0;
    virtual int doLeftControl(State state) = 0;
    virtual int doRightControl(State state) = 0;
    virtual int getLeftCommand() = 0;
    virtual int getRightCommand() = 0;
};


#endif
