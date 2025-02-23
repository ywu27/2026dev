#include "Elevator.h"
void Elevator::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.6, 0, 0.0);
    config.SmartCurrentLimit(20);
    motor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void Elevator::setState(int state) { // Goes from 0 to 5
    if (state == 0) {
        elevatorCTR.SetReference(starting_SP, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }

    else if (state == 1){
        elevatorCTR.SetReference(level1, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }

    else if (state == 2){
        elevatorCTR.SetReference(level2, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }

    else if (state == 3){
        elevatorCTR.SetReference(level3, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }

    else if (state == 4){
        elevatorCTR.SetReference(level4, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }

    else if (state == 5){
        elevatorCTR.SetReference(cstation, rev::spark::SparkLowLevel::ControlType::kPosition);
        currentState = state;
    }
}

void Elevator::disable(){
    motor.StopMotor();
}