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
    
    double reefTagHeight = 6.875; // Measured in inches
    double coralStationTagHeight = 53.25; // Measured in inches
    double processorTagHeight = 45.875; // Measured in inches
    double bargeTagHeight = 69.0; // Measured in inches
    
    std::vector<int> redReefTargetIDs = {10, 9, 8, 7, 6, 11};
    std::vector<int> blueReefTargetIDs = {21, 22, 17, 18, 19, 20};
    std::vector<double> reefTargetAngles = {0, 60, 120, 180, 240, 300};

    std::vector<int> redCoralStationTargetIDs = {1, 2};
    std::vector<int> blueCoralStationTargetIDs = {12, 13};

    int redProcessorTargetIDs = 3;
    int blueProcessorTargetIDs = 16;

    std::vector<int> redBargeTargetIDs = {4, 14};
    std::vector<int> blueBargeTargetIDs = {5, 15};



    //double coralStationtargetAngle


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

    // sets the tag height based on the target ID by comparing it to the vectors of target IDs in different positions of the field
    // returns the tag height in inches
    int setTagHeight(){
        
        double tagHeight; //Measured in inches
        int fieldElement = getTagID();

        if (std::find(redReefTargetIDs.begin(), redReefTargetIDs.end(), fieldElement) != redReefTargetIDs.end()
        or (std::find(blueReefTargetIDs.begin(), blueReefTargetIDs.end(), fieldElement) != blueReefTargetIDs.end())){

            tagHeight = reefTagHeight;
        }
            
        else if (std::find(redCoralStationTargetIDs.begin(), redCoralStationTargetIDs.end(), fieldElement) != redCoralStationTargetIDs.end()
        or std::find(blueCoralStationTargetIDs.begin(), blueCoralStationTargetIDs.end(), fieldElement) != blueCoralStationTargetIDs.end()) {
                
            tagHeight = coralStationTagHeight;
        }
            
        else if (fieldElement == redProcessorTargetIDs or fieldElement == blueProcessorTargetIDs) {
            
            tagHeight = processorTagHeight;
        }
            
        else if (std::find(redBargeTargetIDs.begin(), redBargeTargetIDs.end(), fieldElement) != redBargeTargetIDs.end()
        or std::find(blueBargeTargetIDs.begin(), blueBargeTargetIDs.end(), fieldElement) != blueBargeTargetIDs.end()) {
            
            tagHeight = bargeTagHeight;

        }
        
        return tagHeight;

    }
    
    // sets the angle setpoint based on the angle of the field element's tag ID
    double setAngleSetpoint(){
        
        int fieldElement = getTagID();
        int angleSetpoint = 0; //Measured in degrees

        if (std::find(redReefTargetIDs.begin(), redReefTargetIDs.end(), fieldElement) != redReefTargetIDs.end()){
            
            //finding the index value of the taget IDs for red reef and using it to find the setpoint angle
            int redIndexAngle = std::find(redReefTargetIDs.begin(), redReefTargetIDs.end(), fieldElement) - redReefTargetIDs.begin();
            angleSetpoint = reefTargetAngles[redIndexAngle];
        }
       
        else if (std::find(blueReefTargetIDs.begin(), blueReefTargetIDs.end(), fieldElement) != blueReefTargetIDs.end()) {
            
            //finding the index value of the taget IDs for blue reef and using it to find the setpoint angle
            int blueIndexAngle = std::find(blueReefTargetIDs.begin(), blueReefTargetIDs.end(), fieldElement) - blueReefTargetIDs.begin();
            angleSetpoint = reefTargetAngles[blueIndexAngle];
        }
            
        else if (std::find(redCoralStationTargetIDs.begin(), redCoralStationTargetIDs.end(), fieldElement) != redCoralStationTargetIDs.end()
        or std::find(blueCoralStationTargetIDs.begin(), blueCoralStationTargetIDs.end(), fieldElement) != blueCoralStationTargetIDs.end()) {
                
            if (fieldElement == 13 or fieldElement == 1){
                angleSetpoint = 45;
            }

            else if (fieldElement == 12 or fieldElement == 2){
                angleSetpoint = -45;
            }
            
        }
            
        else if (fieldElement == redProcessorTargetIDs or fieldElement == blueProcessorTargetIDs) {
            
            angleSetpoint = 90;
        }
            
        else if (std::find(redBargeTargetIDs.begin(), redBargeTargetIDs.end(), fieldElement) != redBargeTargetIDs.end()
        or std::find(blueBargeTargetIDs.begin(), blueBargeTargetIDs.end(), fieldElement) != blueBargeTargetIDs.end()) {
            
            angleSetpoint = 180;

        }
        
        return angleSetpoint;
    }
    
    double getDistanceToWall() { // perpendicular distance to wall in meters
        //if (isTargetDetected() == true)
        //{
            double tagHeight = setTagHeight();
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