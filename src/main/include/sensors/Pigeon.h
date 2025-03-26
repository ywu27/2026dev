#pragma once

#include "ctre/phoenix6/Pigeon2.hpp"
#include <math.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "geometry/Rotation2d.h"

class Pigeon 
{
private:
    ctre::phoenix6::configs::Pigeon2Configuration currentConfiguration{};
    Rotation2d angleOffset = Rotation2d(0.0);

public:
    ctre::phoenix6::hardware::Pigeon2 pigeon;

    Pigeon(int deviceID) : pigeon(ctre::phoenix6::hardware::Pigeon2(deviceID, "Drivetrain"))
    {
    }

    ctre::phoenix6::configs::Pigeon2Configuration getConfig()
    {
        return currentConfiguration;
    }

    void init() {
        ctre::phoenix6::configs::Pigeon2Configuration pigeonConfigs{};

        pigeonConfigs.MountPose.MountPosePitch = units::degree_t(0.0);
        pigeonConfigs.MountPose.MountPoseRoll = units::degree_t(0.0);
        pigeonConfigs.MountPose.MountPoseYaw = units::degree_t(0.0);

        pigeonConfigs.Pigeon2Features.EnableCompass = true;
        pigeonConfigs.FutureProofConfigs = false;

        pigeon.GetConfigurator().Apply(pigeonConfigs);
        currentConfiguration = pigeonConfigs;

        angleOffset = Rotation2d(0.0);
    }

    void setYaw(double desiredYaw) {
        double currentYaw = pigeon.GetAngle();
        angleOffset = Rotation2d::fromDegrees(currentYaw - desiredYaw);
    }

    void setOffset(Rotation2d angleOffsetInput) 
    {
        angleOffset = angleOffsetInput;
    }

    /**
     * Radians
     * Counter clockwise(idk why this is stupid)
     */
    Rotation2d getBoundedAngleCCW()
    {
        return Rotation2d(Rotation2d::degreesBound(-pigeon.GetAngle() - angleOffset.getDegrees()) * PI / 180);
    }

    Rotation2d getBoundedAngleCW()
    {
        return Rotation2d(Rotation2d::degreesBound(pigeon.GetAngle() + angleOffset.getDegrees()) * PI / 180);
    }

    frc::Rotation2d getRotation2d()
    {
        return frc::Rotation2d(units::radian_t((pigeon.GetAngle() + angleOffset.getDegrees()) * PI / 180));
    }
};