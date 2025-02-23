#include "Climber.h"

void Climber::init(){
    config.Inverted(false);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    config.closedLoop.Pid(0.2, 0.0005, 0.0);
    
    //This value will be changed based on testing
    config.SmartCurrentLimit(20);
    climberMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
}

void Climber::disable() {
    climberMotor.StopMotor();
}

void Climber::setVelocity(double speed){
    velocity = speed;
}

void Climber::velocitySetpoint() {
    climberCTR.SetReference(velocity, rev::spark::SparkBase::ControlType::kVelocity);
}