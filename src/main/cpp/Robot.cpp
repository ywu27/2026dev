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
  mGyro.init();
  // mGyro.setYaw(180);
  // frc::CameraServer::StartAutomaticCapture();
  limelight1.setPipelineIndex(0);
  // limelight2.setPipelineIndex(0);

  // Choosers
  allianceChooser.SetDefaultOption("Red Alliance", redAlliance);
  allianceChooser.AddOption("Blue Alliance", blueAlliance);
  frc::SmartDashboard::PutData("Alliance Color", &allianceChooser);

  // Determines alliance color
  std::string allianceColor = allianceChooser.GetSelected();
  if (allianceColor == "RED") {
    // alliance = Limelight::Alliance::RED;
    allianceIsRed = true;
  }
  else {
    // alliance = Limelight::Alliance::BLUE;
    allianceIsRed = false;
  }
  
  // limelight2 = Limelight("two", alliance);

  positionChooser.SetDefaultOption(kAutoStartDefault, kAutoStartDefault);
  positionChooser.AddOption(kAutoStartB, kAutoStartB);
  positionChooser.AddOption(kAutoStartC, kAutoStartC);
  positionChooser.AddOption(kSimpleAuto, kSimpleAuto);
  frc::SmartDashboard::PutData("Auto Start Position", &positionChooser);

  reefChooser.SetDefaultOption(kAutoReefDefault, kAutoReefDefault);
  reefChooser.AddOption(kAutoReefB, kAutoReefB);
  reefChooser.AddOption(kAutoReefC, kAutoReefC);
  reefChooser.AddOption(kAutoReefD, kAutoReefD);
  reefChooser.AddOption(kAutoReefE, kAutoReefE);
  reefChooser.AddOption(kAutoReefF, kAutoReefF);
  reefChooser.AddOption(kOneCoral, kOneCoral);
  frc::SmartDashboard::PutData("Auto Reef Position", &reefChooser);
}

void Robot::RobotPeriodic()
{
  frc::SmartDashboard::PutBoolean("Limelight get target", limelight1.isTargetDetected2());
  frc::SmartDashboard::PutBoolean("Limelight aligned? ", align.isAligned(limelight1));
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();

  align.forwardPID.Reset();
  align.strafePID.Reset();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));
  std::string start_pos = positionChooser.GetSelected();
  std::string reef_pos = reefChooser.GetSelected();

  // Auto path choosing
  if(start_pos=="1" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_1A, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_1B, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_1C, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_1D, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_1E, allianceIsRed);
  }
  else if(start_pos=="1" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_1F, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_2A, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_2B, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_2C, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_2D, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_2E, allianceIsRed);
  }
  else if(start_pos=="2" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_2F, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="A") {
    mTrajectory.followPath(Trajectory::auto_3A, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="B") {
    mTrajectory.followPath(Trajectory::auto_3B, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="C") {
    mTrajectory.followPath(Trajectory::auto_3C, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="D") {
    mTrajectory.followPath(Trajectory::auto_3D, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="E") {
    mTrajectory.followPath(Trajectory::auto_3E, allianceIsRed);
  }
  else if(start_pos=="3" && reef_pos=="F") {
    mTrajectory.followPath(Trajectory::auto_3F, allianceIsRed);
  }
  else if (start_pos == "0" && reef_pos == "0") {
    mTrajectory.followPath(Trajectory::MOVE_STRAIGHT, allianceIsRed);
  }
}
void Robot::AutonomousPeriodic()
{
  // mDrive.enableModules();
  // ChassisSpeeds speeds = align.autoAlign(limelight1, 1, 0);
  // float vx = speeds.vxMetersPerSecond;
  // float vy = speeds.vyMetersPerSecond;
  // float rot = 0.0;
  // bool fieldOriented = false;

  // mDrive.Drive(
  //     ChassisSpeeds(vx, vy, rot),
  //     mGyro.getBoundedAngleCCW(),
  //     fieldOriented,
  //     false);
  // mDrive.updateOdometry();
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;
  // mGyro.init();
  mDrive.enableModules();
  mDrive.resetOdometry(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));
  holdTimer.Reset();

  mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
  xStickLimiter.reset(0.0);
  yStickLimiter.reset(0.0);

  align.forwardPID.Reset();
  align.strafePID.Reset();
}

void Robot::TeleopPeriodic()
{
  float currentScale = mPowerModule.currentScale();

  if (ctr.GetL2Button()) {
    speedLimiter = 0.2;
  }
  else {
    speedLimiter = 1.0;
  }

  bool fieldOriented = mGyro.gyro.IsConnected();

  auto startTime = frc::Timer::GetFPGATimestamp();
  double vx = 0;
  double vy = 0;

  // Controller inputs
  double leftX = ControlUtil::deadZonePower(ctr.GetLeftX(), ctrDeadzone, 1);
  double leftY = ControlUtil::deadZonePower(-ctr.GetLeftY(), ctrDeadzone, 1);

  leftX = xStickLimiter.calculate(leftX) * speedLimiter * currentScale; 
  leftY = yStickLimiter.calculate(leftY) * speedLimiter * currentScale;

  double rightX = speedLimiter * ControlUtil::deadZoneQuadratic(ctr.GetRightX(), ctrDeadzone);
  double rot = 0;

  // Driver
  int dPad = ctr.GetPOV();
  // bool rumbleController = false; //ADD THIS
  bool alignLimelight = ctr.GetR2Button();

  // Driving Modes
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;

  // frc::SmartDashboard::PutNumber("transY", transY);
  // frc::SmartDashboard::PutNumber("transX", transX);
  frc::SmartDashboard::PutBoolean("At setpoint forward", align.forwardPID.AtSetpoint());
  frc::SmartDashboard::PutBoolean("At setpoint side", align.strafePID.AtSetpoint());
  // frc::SmartDashboard::PutNumber("desired setpoint", transY);
  frc::SmartDashboard::PutNumber("current setpoint", mDrive.getOdometryPose().Y().value());

  if (ctr.GetR2ButtonPressed()) {
    align.forwardPID.Reset();
    align.strafePID.Reset();
    Pose3d robotPose = limelight1.getRobotPoseFieldSpace();
    Pose3d aprilTagPose = limelight1.getTargetPoseRobotSpace(); // In meters
    float apriltagPoseFeetX = aprilTagPose.x; // Convert to feet
    float apriltagPoseFeetY = aprilTagPose.y; // Convert to feet
    transY = mDrive.getOdometryPose().Y().value() + 3.0;
    transX = mDrive.getOdometryPose().X().value() + 3.0;
  }
  else if (alignLimelight) { // Alignment Mode // LL1 is reef
    // if(limelight1.isTargetDetected2()){
      // if (limelight2.getTagType()==Limelight::REEF) {
      //   offSet = 0.0381; // meters
      // }
      targetDistance = 1; //set this
      zeroSetpoint = 0;
      // zeroSetpoint = limelight1.getAngleSetpoint();
      // ChassisSpeeds speeds = align.driveToSetpointY(transY, mDrive, mGyro);
      ChassisSpeeds speeds = align.autoAlign(limelight1, targetDistance, offSet);
      vx = speeds.vxMetersPerSecond;
      vy = speeds.vyMetersPerSecond;
      fieldOriented = false;
      mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
      mHeadingController.setSetpoint(zeroSetpoint);
      rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
    // }
    // if(limelight2.isTargetDetected2()){ // Alignment Mode // LL2 is coral station
    //   if (limelight2.getTagType()==Limelight::REEF) {
    //     offSet = 0.0381; // meters
    //   }
    //   targetDistance = 0.5; // set this
    //   zeroSetpoint = limelight2.getAngleSetpoint();
    //   ChassisSpeeds speeds = align.autoAlign(limelight2, targetDistance, offSet);
    //   vx = speeds.vxMetersPerSecond;
    //   vy = speeds.vyMetersPerSecond;
    //   fieldOriented = false;
    //   mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
    //   mHeadingController.setSetpoint(zeroSetpoint);
    //   rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
    // } 
  }
  // else if (dPadOperator!=-1) { // Snap mode, CHANGE BACK TO ELSEIF ONCE LL MOUNTED
  //   zeroSetpoint = dPadOperator;
  //   mHeadingController.setHeadingControllerState(SwerveHeadingController::SNAP);
  //   mHeadingController.setSetpoint(zeroSetpoint);
  //   rot = mHeadingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
  // }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
    rot = rightX * moduleMaxRot * 2;
  }

  // rot = (rot == 0) ? mFsGyro.autoRot(leftX, leftY, rightX, mGyro) : rightX * moduleMaxRot * 2; // TEST THIS
  
  // Gyro Resets
  if (ctr.GetCrossButtonReleased()) {
    mGyro.init();
  }
  if(ctr.GetTriangleButton()) {
    mDrive.autoRot();
  }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, rot),
      mGyro.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);
  mDrive.updateOdometry();

  // Brownouts
  // frc::SmartDashboard::PutNumber("Power Scaled?", currentScale);
  
  // Smart Dashboard Info
  frc::SmartDashboard::PutBoolean("Limelight get target", limelight1.isTargetDetected2());
  frc::SmartDashboard::PutNumber("Gyro Position", mGyro.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("vx", vx);
  frc::SmartDashboard::PutNumber("vy", vy);
  frc::SmartDashboard::PutNumber("rot", rot);
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());
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