#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"

class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0.5, 0};
    frc::PIDController strafePID{0.9, 0, 0.1};

public:

    ChassisSpeeds autoAlign(Limelight& limelight, SwerveHeadingController& headingController, double distance, bool enableStrafing) { // distance in meters
        ChassisSpeeds speeds;
        //if (limelight.isTargetDetected()) {
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
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-forwardSpeed, strafeSpeed, rotationSpeed);
        //}
        //else {
        //    speeds = ChassisSpeeds(0, 0, 0);
        //}

        return speeds;
    }
};