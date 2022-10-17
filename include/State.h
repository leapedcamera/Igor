#ifndef _STATE_
#define _STATE_

struct State
{
    double ypr[3];
    double angVel[3];
    double leftWheelSpeed;
    double rightWheelSpeed;
    double speed;
};

static double getPitch(State state)
{
    return state.ypr[1];
}
static double getLeftWheelSpeed(State state)
{
    return state.leftWheelSpeed;
}
 static double getRightWheelSpeed(State state)
{
    return state.rightWheelSpeed;
}
static double getSpeed(State state)
{
    return state.speed;
}

#endif