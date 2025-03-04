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
  //frc::CameraServer::StartAutomaticCapture(); UNCOMMENT LATER
}

void Robot::RobotPeriodic()
{
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();
  mSuperstructure.enable();
 
  // if (frc::DriverStation::IsDSAttached()) {
  //   mTraj.isRed = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed; // checks for alliance color
  // }
  // if (limelight1.targetDetected()) {
  //   mTraj.startPose = limelight1.getRobotPoseFieldSpace();
  //   mTraj.receivedPose = true;
  // } else {
  //   mTraj.receivedPose = false;
  // }
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

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);
}

void Robot::TeleopPeriodic()
{
  bool fieldOriented = mGyro.gyro.IsConnected();

  auto startTime = frc::Timer::GetFPGATimestamp();
  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);
  leftX = xStickLimiter.calculate(leftX); 
  leftY = yStickLimiter.calculate(leftY);
  double rightX = ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot = 0;

  // Driver
  int dPad = ctr.GetPOV();
  bool rumbleController = false; //ADD THIS
  bool alignLimelight = ctr.GetR2Button();

  bool intakeAlgae = ctr.GetCircleButtonPressed();
  bool intakeCoral = ctr.GetSquareButtonPressed();
  bool scoreCoral = ctr.GetCrossButtonPressed(); // TEST THIS
  bool scoreAlgae = ctr.GetCrossButtonPressed();

  bool elevatorUp = ctr.GetR1ButtonPressed();
  bool elevatorDown = ctr.GetR2ButtonPressed();		
  
  // Co-driver
  bool stowClimber = ctrOperator.GetCircleButtonPressed();
  bool setClimberSetpoint = ctrOperator.GetTriangleButtonPressed();
  bool climb = ctrOperator.GetSquareButton();
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

  if (intakeCoral) {
    mSuperstructure.intakeCoral();
  }
  else if (intakeAlgae) {
    mSuperstructure.controlIntake(1);
  }
  else if (elevatorUp) {
    mSuperstructure.elevatorUp();
  }
  else if (elevatorDown) {
    mSuperstructure.elevatorDown();
  }
  else if (scoreCoral) {
    mSuperstructure.scoreCoral();
  }
  else if (scoreAlgae) {
    mSuperstructure.controlIntake(2);
  }
  else if (setClimberSetpoint) {
    mSuperstructure.controlClimber(1);
  }
  else if (climb) {
    mSuperstructure.controlClimber(2);
  }
  else {

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