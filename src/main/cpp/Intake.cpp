#include "Intake.h"

void Intake::init() {
    intakeConfig.Inverted(false);
    intakeConfig.SmartCurrentLimit(30);
    intakeConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake); // TRY WITH COAST / don't know design
    intakeConfig.closedLoop.Pid(0.2, 0, 0.05);

    angleConfig.Inverted(false);
    angleConfig.SmartCurrentLimit(30);
    angleConfig.SetIdleMode(rev::spark::SparkMaxConfig::IdleMode::kBrake);
    angleConfig.closedLoop.Pid(0.2, 0, 0.05);

    angleMotor.Configure(angleConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);
    intakeMotor.Configure(intakeConfig, rev::spark::SparkMax::ResetMode::kResetSafeParameters, rev::spark::SparkMax::PersistMode::kPersistParameters);

    angleEnc.SetPosition(0);
    intakeEnc.SetPosition(0);
}

void Intake::disable() {
    intakeMotor.StopMotor();
}

void Intake::setSpeed(double speed) {
    intakeSpeed = speed;
}

void Intake::setState(intakeState state) {
    switch (state) {
    case IN:
        // intakeCtr.SetReference(intakeSpeed, rev::spark::SparkBase::ControlType::kVelocity);
        intakeMotor.Set(-0.5);
        break;
    case HOLD:
        hold();
        break;
    case CLEAR:
        clear();
        break;
    case STOP:
        disable();
    }
}

void Intake::setAngle(intakeAngle angle) {
    switch (angle) {
    case UP:
        //test position
        angleCtr.SetReference(0.5, rev::spark::SparkLowLevel::ControlType::kPosition);
        break;
    case DOWN:
        angleCtr.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition);
        break;
    }
}

void Intake::hold() {
    intakeMotor.Set(-0.02);
}

void Intake::clear() {
    double motorVelocity = intakeEnc.GetVelocity();
    double motorCurrentDraw = intakeMotor.GetOutputCurrent();
    intakeCtr.SetReference(0, rev::spark::SparkLowLevel::ControlType::kPosition);
    // intakeMotor.Set(0.5);
    
    // if (timerStarted && timer.Get().value() > 1.0){
    //   intakeMotor.Set(0);
    //   timer.Stop();
    //   timer.Reset();
    //   timerStarted = false;
    // }
  
    // if (timerStarted != true) {
    //     timer.Reset();
    //     timer.Start();
    // }

    // timerStarted = true;
}