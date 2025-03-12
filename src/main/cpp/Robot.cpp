// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <string>
#include <cameraserver/CameraServer.h>

void Robot::RobotInit()
{
  mDrive.initModules();
  mSuperstructure.init();
  mGyro.init();
  //frc::CameraServer::StartAutomaticCapture();
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();
  mSuperstructure.enable();
}
void Robot::AutonomousPeriodic()
{
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;
  mDrive.enableModules();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));
  mSuperstructure.enable();
  holdTimer.Reset();

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}

void Robot::TeleopPeriodic()
{
  frc::SmartDashboard::PutNumber("elevator encoder", mSuperstructure.mElevator.motor.GetEncoder().GetPosition());

  double speedLimiter = mSuperstructure.speedLimiter();
  double climberLimiter = 1;

  if (ctr.GetL2Button()) {
    climberLimiter = 0.2;
  }
  else {
    climberLimiter = 1;
  }

  if (mSuperstructure.mElevator.limitSwitch.Get()) {
    mSuperstructure.mElevator.disable();
  }

  bool fieldOriented = mGyro.gyro.IsConnected();

  auto startTime = frc::Timer::GetFPGATimestamp();
  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = climberLimiter * xStickLimiter.calculate(leftX) * speedLimiter; 
  leftY = climberLimiter * yStickLimiter.calculate(leftY) * speedLimiter;

  double rightX = climberLimiter * ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot = 0;

  // Driver
  int dPad = ctr.GetPOV();
  bool rumbleController = false; //ADD THIS
  bool alignLimelight = ctr.GetR2Button();

  bool intakeAlgae = ctr.GetCircleButton();
  bool scoreAlgae = ctr.GetSquareButton();
  bool scoreCoral = ctr.GetCrossButton(); // TEST THIS
  bool intakeCoral = ctr.GetTriangleButton();

  bool elevatorUp = ctr.GetR1ButtonPressed();
  bool elevatorDown = ctr.GetL1ButtonPressed();
  
  // Co-driver
  bool stowClimber = ctrOperator.GetCircleButtonPressed();
  // bool setClimberSetpoint = ctrOperator.GetTriangleButtonPressed();
  bool climb = ctrOperator.GetSquareButton();
  bool reverseClimb = ctrOperator.GetTriangleButton();
  int dPadOperator = ctrOperator.GetPOV();

  // Driving Modes
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;

  if (alignLimelight && limelight1.isTargetDetected2()) { // Alignment Mode
    if (limelight1.getTagType()==Limelight::REEF) {
      offSet = 0.0381; // meters
    }
    targetDistance = 1;
    zeroSetpoint = limelight1.getAngleSetpoint();
    ChassisSpeeds speeds = align.autoAlign(limelight1, targetDistance, offSet);
    vx = speeds.vxMetersPerSecond;
    vy = speeds.vyMetersPerSecond;
    fieldOriented = false;
    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  }
  else if (alignLimelight && limelight2.isTargetDetected2()) { // Alignment Mode
    if (limelight2.getTagType()==Limelight::REEF) {
      offSet = 0.0381; // meters
    }
    targetDistance = 1;
    zeroSetpoint = limelight2.getAngleSetpoint();
    ChassisSpeeds speeds = align.autoAlign(limelight2, targetDistance, offSet);
    vx = speeds.vxMetersPerSecond;
    vy = speeds.vyMetersPerSecond;
    fieldOriented = false;
    mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  }
  else if (dPadOperator!=-1) { // Snap mode
    zeroSetpoint = dPadOperator;
    mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
    mHeadingController.setSetpoint(zeroSetpoint);
    rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
    rot = rightX * moduleMaxRot * 2;
  }
  
  // Gyro Resets
  if (ctrOperator.GetCrossButtonReleased()) {
    mGyro.init();
  }
  if (align.isAligned(limelight1) || align.isAligned(limelight2)) {
    mGyro.setYaw(zeroSetpoint);
    // scoreCoral = true; // TEST THIS
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, -rot),
      mGyro.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);
  mDrive.updateOdometry();

  if (mSuperstructure.mEndEffector.currentState == EndEffector::AIM || mSuperstructure.mEndEffector.currentState == EndEffector::SCORE) {
    mSuperstructure.mIntake.setState(Intake::STOP);
  }

  if (intakeCoral) {
    mSuperstructure.intakeCoral();
  }
  else if (elevatorUp) {
    mSuperstructure.elevatorUp(false);
    // if (elevatorUp && !ctr.GetL2Button()) {
    //   holdTimer.Start();
    //   // While dPad is elevatorUp
    //   while (elevatorUp) {
    //     elevatorUp = (ctr.GetPOV() == 0);
    //     // If the timer has been doing for more than second, go to level 4
    //     if (holdTimer.Get().value() > 1.0) {
    //       mSuperstructure.mElevator.setState(4, false);
    //       break;
    //     }
    //     frc::SmartDashboard::PutNumber("dPad val", ctr.GetPOV());
    //     frc::SmartDashboard::PutNumber("Hold Timer", holdTimer.Get().value());
    //   }
    //   // else move up one level
    //   if (holdTimer.Get().value() < 1.0) {
    //     mSuperstructure.elevatorUp(false);
    //   }
    //   holdTimer.Stop();
    //   holdTimer.Reset();
    // }
    // else if (ctr.GetL2Button() && elevatorUp) {
    //   mSuperstructure.elevatorUp(true);
    // }
  }
  else if (elevatorDown) {
    mSuperstructure.elevatorDown(false);
    // if (elevatorDown && !ctr.GetL2Button()) {
    //   holdTimer.Start();
    //   // Check if dPad is telling the elevator to go down
    //   while (elevatorDown) {
    //     elevatorDown = (ctr.GetPOV() == 180);
    //     // Check if dPad has been in the down value for a second, if so, go to level 1
    //     if (holdTimer.Get().value() > 1.0) {
    //       mSuperstructure.mElevator.setState(1, false);
    //       break;
    //     }
    //     frc::SmartDashboard::PutNumber("dPad val", ctr.GetPOV());
    //     frc::SmartDashboard::PutNumber("Hold Timer", holdTimer.Get().value());
    //   }
    //   // else just move down one level
    //   if (holdTimer.Get().value() < 1.0) {
    //     mSuperstructure.elevatorDown(false);
    //   }
    //   holdTimer.Stop();
    //   holdTimer.Reset();
    // }
    // else if (elevatorDown && ctr.GetL2Button()) {
    //   mSuperstructure.elevatorDown(true);
    // }
  }
  else if (scoreCoral) {
    mSuperstructure.scoreCoral();
  }
  // else if (setClimberSetpoint) {
  //   mSuperstructure.controlClimber(1);
  // }
  else if (climb) {
    mSuperstructure.controlClimber(2); // climb
  }
  else if (reverseClimb) {
    mSuperstructure.controlClimber(3); // reverse
  }
  else if(intakeAlgae && !(mSuperstructure.mIntake.cSensor.isTarget())) {
    mSuperstructure.mIntake.setState(Intake::IN);
  }
  else if (scoreAlgae) {
    mSuperstructure.mIntake.setState(Intake::CLEAR);
  }
  else if (mSuperstructure.mIntake.cSensor.isTarget()){
    mSuperstructure.mIntake.setState(Intake::HOLD);
  }
  else {
    mSuperstructure.mIntake.intakeMotor.Set(0);
    mSuperstructure.mIntake.angleMotor.Set(0);
  }
  
  // Smart Dashboard Info
  frc::SmartDashboard::PutNumber("Gyro Position", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutBoolean("aligned?", align.isAligned(limelight1));
  frc::SmartDashboard::PutNumber("vx", vx);
  frc::SmartDashboard::PutNumber("vy", vy);
  frc::SmartDashboard::PutNumber("rot", rot);
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());

  frc::SmartDashboard::PutNumber("tx1", limelight1.getTX());
  frc::SmartDashboard::PutNumber("ty1", limelight1.getTX());
  frc::SmartDashboard::PutNumber("tx2", limelight2.getTY());
  frc::SmartDashboard::PutNumber("ty2", limelight2.getTY());
  frc::SmartDashboard::PutNumber("distance1", limelight1.getDistanceToWall());
  frc::SmartDashboard::PutNumber("distance2", limelight2.getDistanceToWall());
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.stopModules();
}
void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}
void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}
void Robot::SimulationPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif