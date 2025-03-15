#pragma once
#include <vector>

class Pose3d
{
public:
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;

    Pose3d(std::vector<double> pose)
    {
        x = pose[0];
        y = pose[1];
        z = pose[2];
        roll = pose[3];
        pitch = pose[4];
        yaw = pose[5];
    }
    Pose3d() {
        x = 0.0;
        y = 0.0;
        z = 0.0;
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
    }
};