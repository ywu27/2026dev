#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"
#include "SwerveModule.h"

class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0, 0.1};
    frc::PIDController strafePID{0.5, 0, 0.1};
    double forwardSpeed = 0;
    double strafeSpeed = 0;
    double targetDistance = 0;
    double currentX = 0;
    double currentY = 0;

public:

    bool isAligned(Limelight& limelight) {
        if (abs(limelight.getTX())<2 && abs(targetDistance-limelight.getDistanceToWall())<0.05) {
            return true;
        }
        return false;
    }

    ChassisSpeeds autoAlign(Limelight& limelight, SwerveHeadingController& headingController, double distance) { // distance in meters
        ChassisSpeeds speeds;
        double tx = limelight.getTX();
        double distanceToTag = limelight.getDistanceToWall();
        targetDistance = distance;
        if (!isAligned(limelight)) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, distance);
            double strafeSpeed = strafePID.Calculate(tx, 0);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-forwardSpeed, -strafeSpeed, 0);
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }

        return speeds;
    }

    ChassisSpeeds driveToSetpoint(double setpointX, double setpointY, SwerveDrive& drive, units::second_t timestamp) { // setpoint in feet
        ChassisSpeeds speeds;
        currentX = drive.m_odometry.GetPose().X().value(); // .mFrontLeft.driveMotorDistance(timestamp);
        currentY = drive.m_odometry.GetPose().Y().value(); // .driveMotorDistance(timestamp);
        if (drive.mFrontLeft.driveMotorDistance(timestamp) < ((abs(setpointX - currentX) <= 0.2) && (abs(setpointY - currentY) <= 0.2)))
        { //abs(setpointX-currentX)>0.2 && abs(setpointY-currentY)>0.2) {
            double strafeSpeed = strafePID.Calculate(currentX, setpointX);
            double forwardSpeed = forwardPID.Calculate(currentY, setpointY);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-1*forwardSpeed, -1*strafeSpeed, 0);
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }
        return speeds;
    }
};