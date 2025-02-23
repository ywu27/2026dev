#pragma once

#include <thread>
#include "Climber.h"
#include "Intake.h"
#include "Elevator.h"

class Superstructure 
{
private:
    std::thread moduleThread;
    bool enableModules;

public:
    Intake mIntake;
    Elevator mElevator;
    Climber mClimber;

    void init();
    void periodic();
    void enable();
    void disable();

    void controlIntake(bool intakeIn, bool intakeClear);
    void controlElevator(std::string dPadState);
    void climb();
};