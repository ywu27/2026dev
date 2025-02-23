#include "Superstructure.h"

void Superstructure::init() {
    mIntake.init();
    mElevator.init();
    mClimber.init();
    mElevator.setState(0);
    mClimber.position(mClimber.STOW);

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

void Superstructure::controlElevator(std::string dPadState) {
    if (dPadState == "Coral") {
        mElevator.setState(5);
    }
    else if (dPadState == "Start") {
        mElevator.setState(0);
    }
    else if (dPadState == "Up") {
        if (mElevator.currentState < 5) {
            mElevator.setState(mElevator.currentState + 1);
        }
    }
    else if (dPadState == "Down") {
        if (mElevator.currentState > 0) {
            mElevator.setState(mElevator.currentState - 1);
        }
    }
}

void Superstructure::climb() {
    mClimber.setVelocity(3000.0); // Change accordingly
    if (mClimber.STOW) {
        mClimber.position(mClimber.STOW);
    }
    else if (mClimber.CLIMB) {
        mClimber.position(mClimber.CLIMB);
        mClimber.climb();
    }
    else {
        mClimber.disable();
    }
}