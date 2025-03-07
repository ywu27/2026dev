#include "EndEffector.h"

void EndEffector::init() {
    scoringConfig.SmartCurrentLimit(5);
    scoringConfig.Inverted(false);
    scoringConfig.closedLoop.Pid(0.2, 0, 0);
    scoringConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angleConfig.SmartCurrentLimit(15);
    angleConfig.Inverted(false);
    angleConfig.closedLoop.Pid(0.2, 0, 0.0);
    angleConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angle2Config.SmartCurrentLimit(15);
    angle2Config.Inverted(true);
    angle2Config.closedLoop.Pid(0.2, 0, 0.0);
    angle2Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    scoringMotor.Configure(scoringConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    angleMotor1.Configure(angleConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    angleMotor2.Configure(angle2Config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    angleEnc1.SetPosition(0.0);
    angleEnc2.SetPosition(0.0);
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
    // scoringCTR.SetReference(5, rev::spark::SparkLowLevel::ControlType::kVelocity);
    angleCTR1.SetReference(2.714, rev::spark::SparkLowLevel::ControlType::kPosition);
    angleCTR2.SetReference(2.714, rev::spark::SparkLowLevel::ControlType::kPosition);
    scoringMotor.Set(-1);
}

void EndEffector::aim() {
    angleCTR2.SetReference(0.1, rev::spark::SparkLowLevel::ControlType::kPosition);
    scoringMotor.Set(0);
}

void EndEffector::score() {
    scoringMotor.Set(1);
    //scoringCTR.SetReference(5, rev::spark::SparkLowLevel::ControlType::kPosition);
}