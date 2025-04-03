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
  pigeon.init();
  
  // frc::CameraServer::StartAutomaticCapture();

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
  //frc::SmartDashboard::PutBoolean("Limelight get target", limelight1.isTargetDetected2());
  //frc::SmartDashboard::PutBoolean("Limelight aligned? ", align.isAligned(limelight1));
  frc::SmartDashboard::PutNumber("Tag ID", camera1.camera.GetLatestResult().GetBestTarget().GetFiducialId());
  
  // visionCache = camera1.returnPoseEstimate();
}

void Robot::AutonomousInit()
{
  mDrive.state = DriveState::Auto;
  mDrive.enableModules();
  pigeon.pigeon.Reset();

  align.forwardPID.Reset();
  align.strafePID.Reset();
  mDrive.resetPoseEstimator(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_rad));
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
}
void Robot::TeleopInit()
{
  mDrive.state = DriveState::Teleop;
  pigeon.pigeon.Reset();
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

  bool fieldOriented = pigeon.pigeon.IsConnected();

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
  double currentAngle = 0;
  double offSet = 0;
  double targetDistance = 0; // CHECK THIS
  double zeroSetpoint = 0;

  // frc::SmartDashboard::PutNumber("transY", transY);
  // frc::SmartDashboard::PutNumber("transX", transX);
  frc::SmartDashboard::PutBoolean("At setpoint forward", align.forwardPID.AtSetpoint());
  frc::SmartDashboard::PutBoolean("At setpoint side", align.strafePID.AtSetpoint());
  // frc::SmartDashboard::PutNumber("desired setpoint", transY);
  frc::SmartDashboard::PutNumber("current setpoint", mDrive.getOdometryPose().Y().value());
  //frc::SmartDashboard::PutNumber("target yaw", camera1.camera.GetLatestResult().GetBestTarget().GetYaw());
  
  if (ctr.GetSquareButtonPressed()) {
    reefSide = "left";
  }
  else if (ctr.GetCircleButtonPressed()) {
    reefSide = "right";
  }

  if (ctr.GetR1ButtonPressed()) {
    align.forwardPID.Reset();
    align.strafePID.Reset();
    mHeadingController.mRotCtr.Reset();
    // Pose3d robotPose = limelight1.getRobotPoseFieldSpace();
    // Pose3d aprilTagPose = limelight1.getTargetPoseRobotSpace(); // In meters
    // float apriltagPoseFeetX = aprilTagPose.x; // Convert to feet
    // float apriltagPoseFeetY = aprilTagPose.y; // Convert to feet
    // transY = mDrive.getOdometryPose().Y().value() + 3.0;
    // transX = mDrive.getOdometryPose().X().value() + 3.0;
  }
  else if (ctr.GetR1Button()) {
    if (reefSide == "left") {
      offSet = -0.25;
    }
    else if (reefSide == "right") {
      offSet = -0.07876;
    }
    double targetYaw = camera1.camera.GetLatestResult().GetBestTarget().GetYaw();
    if (camera1.isTargetDetected() && reefSide == "right") {
      if (!align.isAligned(camera1)) {
        mHeadingController.setSetpoint(camera1.getYaw());
        targetDistance = 0.15; //set this
        ChassisSpeeds speeds = align.autoAlign2(camera1, targetDistance, offSet);
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        fieldOriented = false;
        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
      }
    }
    else if (camera1.isTargetDetected() && reefSide == "left") {
      if (!align.isAligned(camera1)) {
        mHeadingController.setSetpoint(camera1.getYaw());
        targetDistance = 0.15; //set this
        ChassisSpeeds speeds = align.autoAlign2(camera1, targetDistance, offSet);
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        fieldOriented = false;
        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
      }
    }
    else if (camera2.isTargetDetected() && camera2.getTagType() == PhotonVision::TagType::CORALSTATION) {
      if (!align.isAligned(camera2)) {
        mHeadingController.setSetpoint(camera2.getYaw());
        targetDistance = 0.6; //set this
        ChassisSpeeds speeds = align.autoAlign2(camera2, targetDistance, 0);
        vx = speeds.vxMetersPerSecond;
        vy = speeds.vyMetersPerSecond;
        fieldOriented = false;
        mHeadingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
        rot = mHeadingController.calculate(pigeon.getBoundedAngleCW().getDegrees());
      }
    }
  }
  else // Normal driving mode
  {
    mHeadingController.setHeadingControllerState(SwerveHeadingController::OFF);
    vx = leftX * moduleMaxFPS;
    vy = leftY * moduleMaxFPS;
    rot = rightX * moduleMaxRot * 2;
  }
  
  // Gyro Resets
  if (ctr.GetCrossButtonReleased()) {
    pigeon.pigeon.Reset();
  }
  // if(ctr.GetTriangleButtonPressed()) {
  //   std::shared_ptr<pathplanner::PathPlannerPath> autoPath;
  //   std::string reefSpot = reefChooser.GetSelected();

  //   // Spot on the reef 
  //   if (reefSpot == "A") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to A");
  //   }
  //   else if (reefSpot == "B") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to B");
  //   }
  //   else if (reefSpot == "C") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to C");
  //   }
  //   else if (reefSpot == "D") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to D");
  //   }
  //   else if (reefSpot == "E") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to E");
  //   }
  //   else if (reefSpot == "F") {
  //     autoPath = PathPlannerPath::fromPathFile("1 to F");
  //   }
    
  //   // Get goal end state 
  //   PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(units::feet_per_second_t(vx), units::feet_per_second_t(vy), units::feet_per_second_t(rot)), mDrive.GetPoseEstimatorPose().Rotation().Radians(), pathConfig);
  //   float x = traj.getEndState().pose.Translation().X().value();
  //   float y = traj.getEndState().pose.Translation().Y().value();
  //   float rot = traj.getEndState().pose.Rotation().Degrees().value();

  //   // Choose photon or swervePoseEstimator pose2d value
  //   if (camera1.camera.GetLatestResult().HasTargets()) {
  //     startPose = camera1.returnPoseEstimate(); // in meters
  //   }
  //   else {
  //     startPose = mDrive.mSwervePose.GetEstimatedPosition(); // in meters
  //   }
  //   frc::Pose2d endPose{units::meter_t(x), units::meter_t(y), units::degree_t(rot)};

  //   path = mTeleopTraj.GeneratePath(startPose, endPose);
  // }
  // else if (ctr.GetTriangleButton()) {
  //   mTrajectory.followTeleop(path, allianceIsRed);
  // }
  // else if (ctr.GetTriangleButtonReleased()) {
  //   mDrive.stopModules(); // Changed to just set drive motor to 0
  // }

  // Drive function
  mDrive.Drive(
      ChassisSpeeds(vx, vy, rot),
      pigeon.getBoundedAngleCCW(),
      fieldOriented,
      cleanDriveAccum);

  // Pose Updates
  mDrive.updateOdometry();
  // mDrive.updatePoseEstimator(*limelight1, frc::Timer::GetFPGATimestamp());

  // Brownouts
  // frc::SmartDashboard::PutNumber("Power Scaled?", currentScale);
  
  // Smart Dashboard Info
  // frc::SmartDashboard::PutBoolean("Limelight get target", limelight1.isTargetDetected2());
  frc::SmartDashboard::PutNumber("Photon Target Y", camera1.camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().Y().value());
  frc::SmartDashboard::PutNumber("Photon Target X", camera1.camera.GetLatestResult().GetBestTarget().GetBestCameraToTarget().X().value());
  frc::SmartDashboard::PutNumber("Gyro Position CW", pigeon.getBoundedAngleCW().getDegrees());
  frc::SmartDashboard::PutNumber("Gyro Position CCW", pigeon.getBoundedAngleCCW().getDegrees());
  frc::SmartDashboard::PutBoolean("At Setpoint Rot", mHeadingController.mRotCtr.AtSetpoint());
  frc::SmartDashboard::PutNumber("rot speed", mHeadingController.rotateToTag(0, pigeon));
  frc::SmartDashboard::PutNumber("vx", vx);
  frc::SmartDashboard::PutNumber("vy", vy);
  frc::SmartDashboard::PutNumber("rot", rot);
  frc::SmartDashboard::PutNumber("driveX", mDrive.getOdometryPose().X().value());
  frc::SmartDashboard::PutNumber("driveY", mDrive.getOdometryPose().Y().value());
}

void Robot::DisabledInit()
{
  mDrive.state = DriveState::Disabled;
  mDrive.disableModules();
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