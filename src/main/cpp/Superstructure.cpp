#include "Superstructure.h"

void Superstructure::init() {
    mIntake.init();
    mElevator.init();
    mClimber.init();
    enableModules = false;
    moduleThread = std::thread(&Superstructure::periodic, this);
}

void Superstructure::periodic() {
    while (true)
    {
        if (!enableModules)
        {
            // Disable modules here
            mIntake.disable();
            mElevator.disable();
            mClimber.disable();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void Superstructure::enable() {
    enableModules = true;
}

void Superstructure::disable() {
    enableModules = false;
}

void Superstructure::controlIntake(bool intakeIn, bool intakeClear) {
    mIntake.setIntakeSpeed(2000.0);
    if (intakeIn) {
        mIntake.setIntakeState(Intake::intakeState::IN);
    }
    else if (intakeClear) {
        mIntake.setIntakeState(Intake::intakeState::CLEAR);
    }
    else {
        mIntake.setIntakeState(Intake::intakeState::STOP);
    }
    frc::SmartDashboard::PutBoolean("Intake?", intakeIn);
}

void Superstructure::controlElevator() {
    // add dPad stuff that will be done later
}

void Superstructure::controlClimber(bool climberDown) {
    mClimber.setVelocity(3000.0); // Change accordingly
    if (climberDown) {
        mClimber.velocitySetpoint();
    }
    else {
        mClimber.disable();
    }
}

void Superstructure::defaultConfig() {
    // Will need to add some things
}