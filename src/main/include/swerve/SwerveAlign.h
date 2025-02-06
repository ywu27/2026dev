#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"

class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0.5, 0};
    frc::PIDController strafePID{0.9, 0, 0.1};
    double targetDistance;

public:

    bool isAligned(Limelight& limelight) {
        if (abs(limelight.getTX())<0.5 && abs(targetDistance-limelight.getDistanceToWall())<0.05) {
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
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-forwardSpeed, strafeSpeed, 0);
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }

        return speeds;
    }
};