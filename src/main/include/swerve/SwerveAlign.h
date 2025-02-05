#pragma once

#include "sensors/Limelight.h"
#include "ChassisSpeeds.h"
#include "SwerveHeadingController.h"


class SwerveAlign {
private:
    frc::PIDController forwardPID{7, 0.5, 0};
    frc::PIDController strafePID{0.9, 0, 0.1};

public:

    ChassisSpeeds autoAlign(Limelight& limelight, SwerveHeadingController& headingController, double rotationSetpoint, double distance) { // rotationSetpoint in degrees / distance in meters
        ChassisSpeeds speeds;
        //if (limelight.isTargetDetected()) {
            double tx = limelight.getTX();
            //double ty = limelight.getTY();
            double distanceToTag = limelight.getDistanceToWall();

            double forwardSpeed = forwardPID.Calculate(distanceToTag, distance);
            double strafeSpeed = strafePID.Calculate(tx, 0);

            headingController.setHeadingControllerState(SwerveHeadingController::ALIGN);
            headingController.setSetpoint(rotationSetpoint);
            double rotation = headingController.calculate(mGyro.getBoundedAngleCW().getDegrees());
            speeds = ChassisSpeeds::fromRobotRelativeSpeeds(-forwardSpeed, strafeSpeed, rotation);
        //}
        //else {
        //    speeds = ChassisSpeeds(0, 0, 0);
        //}

        return speeds;
    }
};