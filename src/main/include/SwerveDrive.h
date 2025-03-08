#pragma once

#include "SwerveModule.h"
#include "geometry/Rotation2d.h"
#include "swerve/ChassisSpeeds.h"
#include "geometry/Pose2d.h"
#include "geometry/Twist2d.h"
#include "geometry/Translation2d.h"
#include "Constants.h"
#include "swerve/SwerveDriveKinematics.h"
#include "swerve/SwerveModuleState.h"
#include <thread>
#include "util/ShuffleUI.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <networktables/NetworkTableEntry.h>
#include "sensors/NavX.h"

#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "pathplanner/lib/path/PathPlannerPath.h"

// TODO: inherit thread helper
enum DriveState{
    Teleop,
    Auto,
    Disabled,
    Test
};
class SwerveDrive
{
public: // put back to private

    SwerveModule mFrontLeft = SwerveModule(FLsteerID, FLdriveID, FL_CAN_ID);
    SwerveModule mFrontRight = SwerveModule(FRsteerID, FRdriveID, FR_CAN_ID);
    SwerveModule mBackLeft = SwerveModule(BLsteerID, BLdriveID, BL_CAN_ID);
    SwerveModule mBackRight = SwerveModule(BRsteerID, BRdriveID, BR_CAN_ID);

    // Threas for each Module
    std::thread modulePIDThread;
    float maxRot = moduleMaxRot;
    std::array<Translation2d, 4> wheelPs = {Translation2d(trackWidthNumber, wheelBase), Translation2d(trackWidthNumber, -wheelBase), Translation2d(-trackWidthNumber, wheelBase), Translation2d(-trackWidthNumber, -wheelBase)};

    SwerveDriveKinematics m_kinematics = SwerveDriveKinematics(wheelPs);
    NavX &mGyro; 

    // wpi lib class ver of kinemactics used to initialize odometry
    frc::SwerveDriveKinematics<4> frckinematics{ 
        frc::Translation2d{0.7239_m, 0.7239_m},
        frc::Translation2d{0.7239_m, -0.7239_m},
        frc::Translation2d{-0.7239_m, 0.7239_m},
        frc::Translation2d{-0.7239_m, -0.7239_m},
    };

    frc::SwerveDriveOdometry<4> m_odometry{
        frckinematics,
        mGyro.getRotation2d(), 
        // might need to edit order of motors (double check)
        {
            mBackLeft.getModulePosition(), 
            mFrontLeft.getModulePosition(), 
            mFrontRight.getModulePosition(), 
            mBackRight.getModulePosition() 
        },
        frc::Pose2d{0_m, 0_m, 0_deg}
    };

    void runModules(); // Private - do not call outside of init

public:

    SwerveDrive(NavX &mGyroInput) : mGyro(mGyroInput) {
    }

    DriveState state = DriveState::Teleop;
    void Drive(ChassisSpeeds desiredSpeeds, Rotation2d fieldRelativeGyro, bool useFieldOriented, bool cleanAccum = false);
    void initModules();
    void enableModules();
    bool stopModules();
    void resetOdometry(frc::Translation2d trans, frc::Rotation2d angle);
    frc::Pose2d getOdometryPose();
    void updateOdometry();
    // void displayDriveTelemetry();
    // void zeroAccumulation();
};