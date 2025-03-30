#pragma once

#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/path/GoalEndState.h>

#include "Constants.h"

class TeleopTrajectory {
private:
    units::meters_per_second_t maxVelocity = 5.450_mps;
    units::meters_per_second_squared_t maxAcceleration = 8.9_mps_sq;
    units::radians_per_second_t maxRadVelocity = 1.5_rad_per_s;
    units::radians_per_second_squared_t maxRadAccel = 13_rad_per_s_sq;

public:
    std::shared_ptr<pathplanner::PathPlannerPath> GeneratePath(frc::Pose2d startPose, frc::Pose2d endPose) {
        // Configure trajectory constraints
        frc::TrajectoryConfig config(maxVelocity, maxAcceleration);

        frc::Trajectory traj = frc::TrajectoryGenerator::GenerateTrajectory(
            startPose,         
            {},                  
            endPose,             
            config               
        );

        std::vector<pathplanner::Waypoint> pathWaypoints;

        for (const auto& state : traj.States()) {
        // Create a PathPlanner waypoint from each state
            pathplanner::Waypoint waypoint{std::nullopt, frc::Translation2d(state.pose.X(), state.pose.Y()), std::nullopt};
            pathWaypoints.push_back(waypoint);
        }

        // Set path constraints from trajectory configuration
        pathplanner::PathConstraints constraints(maxVelocity, maxAcceleration, maxRadVelocity, maxRadAccel);

        std::shared_ptr<pathplanner::PathPlannerPath> path = std::make_shared<pathplanner::PathPlannerPath>(pathWaypoints, constraints, std::nullopt, pathplanner::GoalEndState(0.0_mps, frc::Rotation2d(endPose.Rotation())));

        return path;
    }
}; 
