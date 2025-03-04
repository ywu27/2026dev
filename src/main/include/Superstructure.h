#pragma once

#include <thread>
#include "Climber.h"
#include "Intake.h"
#include "Elevator.h"
#include "EndEffector.h"

class Superstructure 
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Intake mIntake;
    Elevator mElevator;
    Climber mClimber;
    EndEffector mEndEffector;

    void init();
    void periodic();
    void enable();
    void disable();
    void intakeCoral();
    void controlIntake(int mode);
    void elevatorUp();
    void elevatorDown();
    void scoreCoral();
    void controlClimber(int position);
};