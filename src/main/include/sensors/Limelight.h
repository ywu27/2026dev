#pragma once

#include <vector>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>
#include <iostream>
#include "LimelightHelpers.h"
#include "geometry/Pose3d.h"
#include "Constants.h"

class Limelight {

private:
    std::string limelightName;
    double limelightMountAngle; // Measured in degrees
    double limelightHeight; // Measured in inches
    double tagHeight = 53; // Measured in inches

public:
    Limelight(std::string name, double mountAngle, double heightOffFloor){
        limelightName = name;
        limelightMountAngle = mountAngle; // degrees
        limelightHeight = heightOffFloor; // inches
    }

    bool isTargetDetected() {
        if ((nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)) == 1)
        {
            frc::SmartDashboard::PutNumber("insidetest", (nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)));
            return true;
        }
        frc::SmartDashboard::PutNumber("insidetest", (nt::NetworkTableInstance::GetDefault().GetTable("")->GetNumber("tv", 0.0)));
        return false;
    }

    void setPipelineIndex(int index) {
        LimelightHelpers::setPipelineIndex("", index);
    }

    int getTagID() { // returns the ID of the AprilTag
        return (int)LimelightHelpers::getFiducialID();
    }

    double getTX() { // TX
        if (isTargetDetected() == true)
        {
            double tx = LimelightHelpers::getTX("");
            frc::SmartDashboard::PutNumber("Tx", tx);
            return tx;
        }
    }

    double getTY() { // TY
        if (isTargetDetected() == true)
        {
            double ty = LimelightHelpers::getTY("");
            frc::SmartDashboard::PutNumber("Ty", ty);
            return ty;
        }
    }

    double isTargetDetected2() {
        return !(getTX()==0);
    }

    double getDistanceToWall() { // perpendicular distance to wall in meters
        //if (isTargetDetected() == true)
        //{
            double ty = LimelightHelpers::getTY("");
            double angleToTagDegrees = limelightMountAngle + ty;
            double angleToTagRadians = angleToTagDegrees * (PI / 180.0);
            double distanceToWall = (tagHeight - limelightHeight) / tan(angleToTagRadians);
            distanceToWall *= 0.0254; //converted to meters
            frc::SmartDashboard::PutNumber("distanceToWall", distanceToWall);
            return distanceToWall;
        //}
        //else {
        //    return 0;
        //}
    }

    double getAngleLimelightToTag() { // TY + limelight mount angle
        if (isTargetDetected() == true)
        {
            double ty = getTY();
            double angleToTagDegrees = limelightMountAngle + ty;
            frc::SmartDashboard::PutNumber("vertical angle", angleToTagDegrees);
            return angleToTagDegrees;
        }
        else
        {
            return 0;
        }
    }

    std::vector<double> getPolarCoords() {
        std::vector<double> polarCoords = {getTX(), getDistanceToWall()};
        return polarCoords;
    }

    std::vector<double> getXYCoords() {
        double angle = getTX() * (PI / 180);
        double x = (getDistanceToWall()) * sin(angle);
        double y = (getDistanceToWall()) * cos(angle);
        std::vector<double> xyCoords = {x, y};
        return xyCoords;
    }

    bool isIn(int object, std::vector<int> inp) { // use to check if current ID is part of target ID list
        for (unsigned int i = 0; i < inp.size(); i++)
        {
            if (inp[i] == object)
            {
                return true;
            }
        }
        return false;
    }

    void setLEDMode(int mode) { // 0 for off / 1 for blink / 2 for on
        nt::NetworkTableInstance::GetDefault().GetTable(limelightName)->PutNumber("ledMode", mode);
    }

    Pose3d getTargetPoseRobotSpace() {
        std::vector<double> x = LimelightHelpers::getTargetPose_RobotSpace();
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }

    Pose3d getRobotPoseFieldSpace() {
        std::vector<double> x = LimelightHelpers::getBotpose();
        Pose3d output = Pose3d(x);
        double tempY = output.y;
        output.y = output.z;
        output.z = -tempY;
        return output;
    }

    double moveAmt() { // inches
        return tan(getTX() * (PI / 180.0)) * getDistanceToWall();
    }
};