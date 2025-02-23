#include "Elevator.h"

void Elevator::init(){
    config
        .Inverted(false)
        .SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop
        .Pid(0.2, 0.0005, 0.0);
    enc.SetPosition(startLevel);
    config.SmartCurrentLimit(20); // change this value based on testing
}

void Elevator::disable() {
    motor.StopMotor();
}

void Elevator::setState(elevatorState state){
    if (state != currentState) 
    {
        switch (state) 
        {
            case START:
                elevatorCTR.SetReference(startLevel, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            case LEVEL1:
                elevatorCTR.SetReference(level1, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            case LEVEL2:
                elevatorCTR.SetReference(level2, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            case LEVEL3:
                elevatorCTR.SetReference(level3, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            case LEVEL4:
                elevatorCTR.SetReference(level4, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            case CORALSTATION:
                elevatorCTR.SetReference(coralStation, rev::spark::SparkBase::ControlType::kPosition);
                currentState = state;
                break;
            
        }

    }
}