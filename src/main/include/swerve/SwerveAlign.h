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
    // double currentX = 0;
    // double currentY = 0;

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

    double calculateTurnAngle(double sideX, double sideY) {
        double pythag = sqrt(pow(sideX, 2) + pow(sideY, 2)); // third side of the triangle
        double angleY = std::asin(sideY / pythag) * (180 / PI); // angle looking at side Y

        if (sideX > 0 && sideY > 0) {
            return 90.0 - angleY;
        }
        else if (sideX < 0 && sideY > 0) {
            return -1*(90.0 - angleY);
        } 
        else if (sideX < 0 && sideY < 0) {
            return -1*(90.0 + angleY);
        }
        else if (sideX > 0 && sideY < 0) {
            return 90.0 + angleY;
        }
    }

    ChassisSpeeds driveToSetpoint(double setpointX, double setpointY, SwerveHeadingController &headingController, SwerveDrive& drive, NavX &mGyro) { 
        // Variables
        ChassisSpeeds speeds;
        double currentY = drive.getOdometryPose().Y().value();

        //Tolerance
        forwardPID.SetTolerance(0.2, 0.01); // Change accordingly
        
        if (!forwardPID.AtSetpoint()) {
            double forwardSpeed = forwardPID.Calculate(currentY, setpointY);
            headingController.setSetpoint(calculateTurnAngle(setpointX, setpointY));
            double rot = headingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(0, -forwardSpeed, rot);
        } else {
            speeds = ChassisSpeeds(0, 0, 0);
        }

        return speeds;
    }
};