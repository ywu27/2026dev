#include "Intake.h"

void Intake::init() {
    config.SmartCurrentLimit(intakeCurrentLimit);
    config.SmartCurrentLimit(clearingCurrentLimit);
    config.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    intakeMotor.Configure(config, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
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
        intakeController.SetReference(intakeSpeed, rev::spark::SparkBase::ControlType::kVelocity);
        break;
    case CLEAR:
        clear();
        break;
    case STOP:
        disable();
    }
}

void Intake::clear() {
    double motorVelocity = intakeEncoder.GetVelocity();
    double motorCurrentDraw = intakeMotor.GetOutputCurrent();
    frc::SmartDashboard::PutBoolean("IntakeClearFlag", false);
    intakeController.SetReference(-intakeSpeed, rev::spark::SparkBase::ControlType::kVelocity);
}