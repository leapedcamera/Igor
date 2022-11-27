#include "PidCascade.h"

volatile bool motorInit = false;

// Control-params
double setpoint = -90;
double sSetpoint = 0;
double lSetpoint = 0;
double rSetpoint = 0;
double speed[3];
int leftOutput;
int rightOutput;
int moveState = 0; //0 = balance; 1 = back; 2 = forth

// Balance gains
double Kp = 16.7;
double Kd = 5;
double Ki = .001;

// Speed Gains
double sKp = 3;
double sKd = 2;
double sKi = .05;

// Left wheel gains
double lKp = 3;
double lKd = 2;
double lKi = .05;

// Right wheel gains
double rKp = 3;
double rKd = 2;
double rKi = .05;

PidCascade::PidCascade():
    RobotController()
{

};

double PidCascade::defaultCascade(State input)
{
    return 0;
};

bool PidCascade::addPid( 
    std::string self, std::string parent, PIDController& pid, const CascadeFunction& function )
{
    this->cascadeMap.emplace( self, CascadeNode(std::make_shared<PIDController>(pid), function, parent, 0, false ));
    return true;
};

void PidCascade::setLeftMode(std::string mode)
{
    this->leftMode = mode;
};

void PidCascade::setRightMode(std::string mode)
{
    this->rightMode = mode;
};

void PidCascade::initControl() 
{
    PIDController anglePid;
    PIDController speedPid;
    PIDController lTurnPid;
    PIDController rTurnPid;
    anglePid.begin();
    anglePid.tune(Kp, Ki, Kd);
    anglePid.setpoint(setpoint);
    anglePid.limit(-255, 255);
    speedPid.begin();
    speedPid.tune(sKp, sKi, sKd);
    speedPid.setpoint(sSetpoint);
    speedPid.limit(-255, 255);
    lTurnPid.begin();
    lTurnPid.tune(lKp, lKi, lKd);
    lTurnPid.setpoint(lSetpoint);
    lTurnPid.limit(-255, 255);
    rTurnPid.begin();
    rTurnPid.tune(rKp, rKi, rKd);
    rTurnPid.setpoint(rSetpoint);
    rTurnPid.limit(-255, 255);
    this->addPid("pitch", "PRIMARY", anglePid, getPitch);
    this->addPid("speed", "pitch", speedPid, getSpeed);
    this->addPid("lTurnPid", "speed", lTurnPid, getLeftWheelSpeed);
    this->addPid("rTurnPid", "speed", rTurnPid, getRightWheelSpeed);
    this->setLeftMode("pitch");
    this->setRightMode("pitch");
};

int PidCascade::getLeftCommand() 
{
    return this->leftCommand;
};

int PidCascade::getRightCommand() 
{
    return this->rightCommand;
};


int PidCascade::doLeftControl(State state) 
{
    double output = 0;
    this->executeNode(this->leftMode, state, output);
    this->leftCommand = output;
    return output;
};

int PidCascade::doRightControl(State state) 
{
    double output = 0;
    this->executeNode(this->rightMode, state, output);
    this->rightCommand = output;
    return output;
};

void PidCascade::refresh()
{
  for( auto& iMap : this->cascadeMap )
  {
    std::get<4>(iMap.second) = false;
  }
}

void PidCascade::executeNode( std::string node, State& state, double& result )
{
    std::string parent = std::get<2>(this->cascadeMap.at(node));

    // If node has already been executed, use 
    if(std::get<4>(this->cascadeMap.at(node)) == true )
    {
        result = std::get<3>(this->cascadeMap.at(node));
        return;
    }
    if( parent != "PRIMARY")
    {
      this->executeNode(parent, state, result);
    }

    auto func = std::get<1>(this->cascadeMap.at(node));
    result = std::get<0>(this->cascadeMap.at(node))->compute(func(state) + result);
    std::get<3>(this->cascadeMap.at(node)) = result;
    std::get<4>(this->cascadeMap.at(node)) = true;
};
