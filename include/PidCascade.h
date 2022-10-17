#ifndef PID_CASCADE
#define PID_CASCADE
#pragma once

#include "RobotController.h"
#include "PIDController.h"
#include "State.h"
#include <map>
#include <string>
#include <functional>
#include <vector>
#include <memory>

// Control
#define MIN_ABS_SPEED 85

// Control-params
extern volatile bool motorInit;
extern double speed[3];
extern int leftOutput;
extern int rightOutput;


typedef std::function<double(State state)> CascadeFunction;
typedef std::tuple<std::shared_ptr<PIDController>, CascadeFunction, std::string, double, bool> CascadeNode;

class PidCascade : public RobotController
{
    public:
    PidCascade();
    static double defaultCascade(State state);
    bool setPrimary( std::string primary );
    bool addPid( std::string self, std::string parent, 
        PIDController& pid, const CascadeFunction& function);
    void setLeftMode(std::string mode);
    void setRightMode(std::string mode);
    void initControl() override;
    int doLeftControl(State state) override;
    int doRightControl(State state) override;
    int getLeftCommand() override;
    int getRightCommand() override;
    void executeNode( std::string node, State& state, double& result );
    
    private:
    std::string leftMode;
    std::string rightMode;
    std::map<std::string, CascadeNode> cascadeMap;
    int leftCommand;
    int rightCommand;

};


#endif