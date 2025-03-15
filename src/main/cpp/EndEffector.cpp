#include "EndEffector.h"

void EndEffector::init() {
    scoringConfig.SmartCurrentLimit(30);
    scoringConfig.Inverted(false);
    scoringConfig.closedLoop.Pid(1, 0, 0.01);
    scoringConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angle1Config.SmartCurrentLimit(30);
    angle1Config.Inverted(false);
    angle1Config.closedLoop.Pid(20, 0.1, 0.1);
    angle1Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angle2Config.SmartCurrentLimit(30);
    angle2Config.Inverted(true);
    angle2Config.closedLoop.Pid(20, 0.1, 0.1);
    angle2Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    scoringMotor.Configure(scoringConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    angleMotor1.Configure(angle1Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    angleMotor2.Configure(angle2Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    angleEnc1.SetPosition(0.0);
    angleEnc2.SetPosition(0.0);
    scoringEnc.SetPosition(0);
}

void EndEffector::disable() {
    scoringMotor.StopMotor();
    angleMotor1.StopMotor();
    angleMotor2.StopMotor();
}

void EndEffector::setVelocity(double speed) {
    velocity = speed;
}

void EndEffector::setState(EndEffectorState state) {
    switch (state) {
    case INTAKE:
        intake();
        currentState = INTAKE;
        break;
    case SCORE:
        score();
        currentState = SCORE;
        break;
    case AIM:
        aim();
        currentState  = AIM;
        break;
    case STOP:
        disable();
        currentState = STOP;
        break;
    }
}

void EndEffector::intake() {
    angleCTR1.SetReference(2.3847, rev::spark::SparkLowLevel::ControlType::kPosition);
    angleCTR2.SetReference(2.3847, rev::spark::SparkLowLevel::ControlType::kPosition);
    scoringMotor.Set(-1);
}

void EndEffector::aim() {
    angleCTR2.SetReference(0.6428, rev::spark::SparkLowLevel::ControlType::kPosition);
    angleCTR1.SetReference(0.6428, rev::spark::SparkLowLevel::ControlType::kPosition);
    scoringMotor.Set(-1);
}

void EndEffector::score() {
    scoringMotor.Set(1);
    //scoringCTR.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition); //TEST THIS
}