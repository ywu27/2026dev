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

// #include "Robot.h"
// #include <frc/PS5Controller.h>

// #include <frc/smartdashboard/SmartDashboard.h>
// #include <wpi/print.h>

// Robot::Robot() {
//   m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
//   m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
//   frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
//   mendeffector.init();
// }

// /**
//  * This function is called every 20 ms, no matter the mode. Use
//  * this for items like diagnostics that you want ran during disabled,
//  * autonomous, teleoperated and test.
//  *
//  * <p> This runs after the mode specific periodic functions, but before
//  * LiveWindow and SmartDashboard integrated updating.
//  */
// void Robot::RobotPeriodic() {}

// /**
//  * This autonomous (along with the chooser code above) shows how to select
//  * between different autonomous modes using the dashboard. The sendable chooser
//  * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
//  * remove all of the chooser code and uncomment the GetString line to get the
//  * auto name from the text box below the Gyro.
//  *
//  * You can add additional auto modes by adding additional comparisons to the
//  * if-else structure below with additional strings. If using the SendableChooser
//  * make sure to add them to the chooser code above as well.
//  */
// void Robot::AutonomousInit() {
//   m_autoSelected = m_chooser.GetSelected();
//   // m_autoSelected = SmartDashboard::GetString("Auto Selector",
//   //     kAutoNameDefault);
//   wpi::print("Auto selected: {}\n", m_autoSelected);

//   if (m_autoSelected == kAutoNameCustom) {
//     // Custom Auto goes here
//   } else {
//     // Default Auto goes here
//   }
// }

// void Robot::AutonomousPeriodic() {
//   if (m_autoSelected == kAutoNameCustom) {
//     // Custom Auto goes here
//   } else {
//     // Default Auto goes here
//   }
// }