#include "Superstructure.h"

void Superstructure::init() {
    mIntake.init();
    mElevator.init();
    mClimber.init();
    mEndEffector.init();
    mElevator.setState(0);
    mClimber.position(Climber::STOW);

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
            mEndEffector.disable();
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

void Superstructure::controlIntake(int mode) { // 0 for stop / 1 for intake / 2 for clear
    mIntake.setIntakeSpeed(2000.0); // CHANGE IF NECESSARY
    if (mode==0) {
        mIntake.setIntakeState(Intake::intakeState::STOP);
    }
    else if (mode==1) {
        mIntake.setIntakeState(Intake::intakeState::IN);
    }
    else if (mode==2) {
        mIntake.setIntakeState(Intake::intakeState::CLEAR);
    }
}

void Superstructure::elevatorUp() {
    if (mElevator.getCurrentState()<4) {
        mElevator.setState(mElevator.getCurrentState() + 1);
    }
    mEndEffector.setState(EndEffector::AIM);
}

void Superstructure::elevatorDown() {
    if (mElevator.getCurrentState()>1) { // TRY ZERO AS WELL
        mElevator.setState(mElevator.getCurrentState() - 1);
    }
    mEndEffector.setState(EndEffector::AIM);
}

void Superstructure::intakeCoral() {
    mElevator.setState(5);
    mEndEffector.setState(EndEffector::INTAKE);
}

void Superstructure::scoreCoral() {
    mEndEffector.setState(EndEffector::SCORE);
}

void Superstructure::controlClimber(int mode) { // 0 for stow / 1 for setpoint / 2 for climbing
    mClimber.setVelocity(3000.0); // CHANGE IF NECESSARY
    if (mode==0) {
        mClimber.position(Climber::STOW);
    }
    else if (mode==1) {
        mClimber.position(Climber::CLIMB);
    }
    else if (mode==2) {
        mClimber.climb();
    }
    else {
        mClimber.disable();
    }
}