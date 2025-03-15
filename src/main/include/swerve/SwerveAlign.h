#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"
#include "SwerveDrive.h"

class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0, 0.1};
    frc::PIDController strafePID{2, 0, 0.1};
    
    double forwardSpeed = 0;
    double strafeSpeed = 0;
    double targetDistance = 0;
    double targetOffset = 0;

public:

    double currentX = 0;
    double currentY = 0;
    double prevErrorX = 0;
    double prevErrorY = 0;

    bool isAligned(Limelight& limelight) {
        if (abs(limelight.getTargetPoseRobotSpace().x-targetOffset)<0.05 && abs(targetDistance-limelight.getDistanceToWall())<0.05) {
            return true;
        }
        return false;
    }

    ChassisSpeeds autoAlign(Limelight& limelight, double setpointDistance, double offsetSetpoint) { // distance in meters
        ChassisSpeeds speeds;
        double offset = limelight.getTargetPoseRobotSpace().x;
        double distanceToTag = limelight.getDistanceToWall();
        targetDistance = setpointDistance;
        targetOffset = offsetSetpoint;
        if (!isAligned(limelight)) {
            double forwardSpeed = forwardPID.Calculate(distanceToTag, setpointDistance);
            double strafeSpeed = strafePID.Calculate(offset, offsetSetpoint);
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-strafeSpeed, -forwardSpeed, 0); //CHECK THIS
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }
        return speeds;
    }

    ChassisSpeeds driveToSetpointX(double setpointX, SwerveDrive& drive, NavX &mGyro) { 
        // Variables
        ChassisSpeeds speeds;
        double currentX = drive.getOdometryPose().X().value();

        //Tolerance
        strafePID.SetTolerance(0.02, 0.01);

        if (!strafePID.AtSetpoint()) {
            double strafeSpeed = strafePID.Calculate(currentX, setpointX);
            if (prevErrorX < (setpointX-currentX)) {
                strafeSpeed = -fabs(strafeSpeed);
            }
            else {
                strafeSpeed = fabs(strafeSpeed);
            }
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(strafeSpeed, 0, 0);
        }
        else {
            strafeSpeed = 0;
            speeds = ChassisSpeeds(0, 0, 0);
        }

        prevErrorX = setpointX-currentX;
        return speeds;
    }

    ChassisSpeeds driveToSetpointY(double setpointY, SwerveDrive& drive, NavX &mGyro) { 
        // Variables
        ChassisSpeeds speeds;
        double currentY = drive.getOdometryPose().Y().value();

        // Tolerance
        forwardPID.SetTolerance(0.02, 0.01);

        if (!forwardPID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(currentY, setpointY);
            if (prevErrorY < (setpointY - currentY)) {
                forwardSpeed = -fabs(forwardSpeed);
            }
            else {
                forwardSpeed = fabs(forwardSpeed);
            }
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(0, forwardSpeed, 0);
        }
        else {
            forwardSpeed = 0;
            speeds = ChassisSpeeds(0, 0, 0);
        }

        prevErrorY = setpointY - currentY;
        return speeds;
    }
};