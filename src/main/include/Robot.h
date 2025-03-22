// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc/PS5Controller.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>

#include "util/ShuffleUI.h"
#include <thread>

#include "util/ControlUtil.h"
#include "sensors/NavX.h"
#include "swerve/SwerveHeadingController.h"
#include "util/TimeDelayedBool.h"
#include <frc/Joystick.h>
#include "sensors/Limelight.h"
#include "util/SlewRateLimiter.h"
#include <frc/GenericHID.h>
#include "SwerveDrive.h"
#include "swerve/SwerveAlign.h"
#include "util/TimeDelayButton.h"
#include "sensors/Limelight.h"
#include <ctre/phoenix6/CANBus.hpp>
#include <frc/GenericHID.h>
#include "Trajectory.h"
#include "control/PowerModule.h"
#include "sensors/FusedGyro.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() override;
  void RobotPeriodic() override;

  void AutonomousInit() override;
  void AutonomousPeriodic() override;

  void TeleopInit() override;
  void TeleopPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  void TestInit() override;
  void TestPeriodic() override;

  void SimulationInit() override;
  void SimulationPeriodic() override;

  frc::PS5Controller ctr = frc::PS5Controller(0);
  frc::PS5Controller ctrOperator = frc::PS5Controller(1);

  NavX mGyro = NavX();
  SwerveDrive mDrive = SwerveDrive(mGyro);
  Limelight::Alliance alliance;
  Limelight limelight1 = Limelight("limelight-one");
  Limelight limelight2 = Limelight("limelight-two");
  float transY = 0.0;
  float transX = 0.0;

  // For Auto Align
  SwerveAlign align;
  pathplanner::RobotConfig pathConfig = pathplanner::RobotConfig::fromGUISettings();
  Trajectory mTrajectory = Trajectory(mDrive, mGyro, pathConfig);

  //CANivore
  ctre::phoenix6::CANBus canbus{"Drivetrain"};
  ctre::phoenix6::CANBus::CANBusStatus canInfo = canbus.GetStatus();
  float busUtil = canInfo.BusUtilization;

  // Fused Gyro
  FusedGyro mFsGyro; 

  // Teleop Controls
  float ctrPercent = 1.0;
  float boostPercent = 0.9;
  double ctrPercentAim = 0.3;
  TimeDelayButton snapRobotToGoal;
  bool scoreAmp = false;
  bool liftElev = false;
  bool cleanDriveAccum = true;
  float speedLimiter = 1.0;

  // Hold Timer for dPad hold
  frc::Timer holdTimer;

  // Power Module
  PowerModule mPowerModule;

  // Controllers
  SwerveHeadingController mHeadingController = SwerveHeadingController(-4.0, 4.0);
  SlewRateLimiter xStickLimiter = SlewRateLimiter(ctrSlewRate);
  SlewRateLimiter yStickLimiter = SlewRateLimiter(ctrSlewRate);

  // Alliance Color and Chooser
  frc::SendableChooser<std::string> allianceChooser;
  const std::string redAlliance = "RED";
  const std::string blueAlliance = "BLUE";
  bool allianceIsRed = false;

  // Field Positions and Chooser
  frc::SendableChooser<std::string> positionChooser;
  const std::string kSimpleAuto = "0";
  const std::string kAutoStartDefault = "1";
  const std::string kAutoStartB = "2";
  const std::string kAutoStartC = "3";

  // Reef Target Position and Chooser
  frc::SendableChooser<std::string> reefChooser;
  const std::string kOneCoral = "0";
  const std::string kAutoReefDefault = "A";
  const std::string kAutoReefB = "B";
  const std::string kAutoReefC = "C";
  const std::string kAutoReefD = "D";
  const std::string kAutoReefE = "E";
  const std::string kAutoReefF = "F";
};