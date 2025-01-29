#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"

#define forwardFactor 10
#define strafeFactor 10

class SwerveAlign {
private:
    static frc::PIDController forwardPID;
    static frc::PIDController strafePID;

    frc::PIDController forwardPID{0.1, 0, 0};
    frc::PIDController strafePID{0.01, 0, 0};

public:

    static ChassisSpeeds SwerveAlign::autoAlign(Limelight& limelight, SwerveHeadingController& headingController, double distance, bool enableStrafing) { // distance in meters
        ChassisSpeeds speeds;
        if (limelight.isTargetDetected()) {
            double tx = limelight.getTX();
            double ty = limelight.getTY();
            double distanceToTag = limelight.getDistanceToWall();
            double desiredAngle = 0.0;
            //headingController.setSetpoint(desiredAngle);
            //double rotationOutput = headingController.calculate(tx);

            double forwardSpeed = forwardPID.Calculate(distanceToTag, distance);
            double strafeSpeed = 0;
            if (enableStrafing) {
              strafeSpeed = strafePID.Calculate(tx, 0);
            }
            double rotationSpeed = 0;
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(forwardSpeed * forwardFactor, strafeSpeed * strafeFactor, rotationSpeed);
        }
        else {
            speeds = ChassisSpeeds(0, 0, 0);
        }

        return speeds;
    }
};