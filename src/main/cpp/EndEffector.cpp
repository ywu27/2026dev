#include "EndEffector.h"

void EndEffector::init() {
    scoringConfig.SmartCurrentLimit(30);
    scoringConfig.Inverted(false);
    scoringConfig.closedLoop.Pid(1, 0, 0.01);
    scoringConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angle1Config.SmartCurrentLimit(35);
    angle1Config.Inverted(false);
    angle1Config.closedLoop.Pid(8, 0.1, 0.1, rev::spark::kSlot0);
    angle1Config.closedLoop.Pid(1, 0, 0.1, rev::spark::kSlot1);
    angle2Config.closedLoop.maxMotion.MaxVelocity(1);
    angle1Config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);

    angle2Config.SmartCurrentLimit(35);
    angle2Config.Inverted(true);
    angle2Config.closedLoop.Pid(8, 0.1, 0.1, rev::spark::kSlot0);
    angle2Config.closedLoop.Pid(0, 0, 0, rev::spark::kSlot1);
    angle2Config.closedLoop.maxMotion.MaxVelocity(1);
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
    angleCTR1.SetReference(2.447, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0);
    angleCTR2.SetReference(2.447, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot0);
    scoringMotor.Set(-0.5);
}

void EndEffector::aim() {
    // angleCTR2.SetReference(0.7428, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot1, -0.2);
    angleCTR1.SetReference(0.7428, rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::kSlot1, -0.2);
    scoringMotor.Set(-0.3);
}

void EndEffector::score() {
    scoringMotor.Set(0.5);
    //scoringCTR.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition); //TEST THIS
}