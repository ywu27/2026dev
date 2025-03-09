#include "Intake.h"

void Intake::init() {
    intakeConfig.Inverted(false);
    intakeConfig.SmartCurrentLimit(10);
    intakeConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); // TRY WITH COAST / don't know design
    intakeConfig.closedLoop.Pid(0.6, 0, 0.0);

    angleConfig.Inverted(false);
    angleConfig.SmartCurrentLimit(10);
    angleConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    angleConfig.closedLoop.Pid(0.6, 0, 0.0);

    angleMotor.Configure(angleConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    intakeMotor.Configure(intakeConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    angleEnc.SetPosition(0);
    intakeEnc.SetPosition(0);
}

void Intake::disable() {
    intakeMotor.StopMotor();
}

void Intake::setIntakeSpeed(double speed) {
    intakeSpeed = speed;
}

void Intake::setIntakeState(intakeState state) {
    switch (state) {
    case IN:
        intakeCtr.SetReference(intakeSpeed, rev::spark::SparkBase::ControlType::kVelocity);
        break;
    case CLEAR:
        clear();
        break;
    case STOP:
        disable();
    }
}

void Intake::clear() {
    double motorVelocity = intakeEnc.GetVelocity();
    double motorCurrentDraw = intakeMotor.GetOutputCurrent();
    intakeCtr.SetReference(-intakeSpeed, rev::spark::SparkBase::ControlType::kVelocity);
}