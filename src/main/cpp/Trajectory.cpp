#include "Trajectory.h"

#include <frc/controller/HolonomicDriveController.h>
#include <frc/kinematics/ChassisSpeeds.h>

// controller used to track trajectories + correct minor disturbances
static frc::HolonomicDriveController controller{
    frc::PIDController{6e-5, 0, 0}, // Change PIDs to be more accurate
    frc::PIDController{6e-5, 0, 0}, // Change PIDs to be more accurate
    frc::ProfiledPIDController<units::radian>{
        0.45, 0, 0,
        frc::TrapezoidProfile<units::radian>::Constraints{
            units::radians_per_second_t(5.0), // prev: 5.0
            units::radians_per_second_squared_t(100)}}}; // prev: 100

/**
 * Drives robot to the next state on trajectory
 * Odometry must be in meters
 */
void Trajectory::driveToState(PathPlannerTrajectoryState const &state)
{
    // Calculate new chassis speeds given robot position and next desired state in trajectory
    frc::ChassisSpeeds const correction = controller.Calculate(mDrive.getOdometryPose(), frc::Pose2d{state.pose.Translation(), state.heading}, state.linearVelocity, state.heading);

    // Calculate x, y speeds from MPS
    double vy_feet = correction.vx.value() * 3.281;
    double vx_feet = correction.vy.value() * 3.281;
// 
    // Clamp rot speed to 2.0 since that is the max rot we allow
    double rot = -std::clamp(correction.omega.value()*0.52, -moduleMaxRot, moduleMaxRot);

    frc::SmartDashboard::PutNumber("autoHeading", state.heading.Degrees().value());
    frc::SmartDashboard::PutNumber("auto odometry x", mDrive.getOdometryPose().X().value());
    frc::SmartDashboard::PutNumber("autoVY", vy_feet);
    frc::SmartDashboard::PutNumber("autoVX", vx_feet);
    frc::SmartDashboard::PutNumber("autoRot", rot);

    mDrive.Drive(ChassisSpeeds{-vx_feet, vy_feet, rot}, mGyro.getBoundedAngleCCW(), true, true);
}

/**
 * Follows pathplanner trajectory
 */
void Trajectory::follow(std::string const &traj_dir_file_path, bool flipAlliance, bool intake, bool first, float startAngle)
{
    mDrive.enableModules();
    auto path = PathPlannerPath::fromPathFile(traj_dir_file_path);

    // switches path to red alliance (mirrors it)
    if (flipAlliance)
    {
        path = path->flipPath();
    }

    PathPlannerTrajectory traj = PathPlannerTrajectory(path, frc::ChassisSpeeds(), 0_rad, config);

    if (first)
    {
        auto const initialState = traj.getInitialState();
        auto const initialPose = initialState.pose.Translation();

        // set second param to initial holonomic rotation
        mDrive.resetOdometry(initialPose, units::angle::degree_t(startAngle));
    }

    frc::Timer trajTimer;
    trajTimer.Start();

    while ((mDrive.state == DriveState::Auto) && (trajTimer.Get() <= traj.getTotalTime()))
    {
        if (intake)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }

        auto currentTime = trajTimer.Get();
        auto sample = traj.sample(currentTime);

        driveToState(sample);
        mDrive.updateOdometry();

        frc::SmartDashboard::PutNumber("curr pose x meters", mDrive.getOdometryPose().Translation().X().value());
        frc::SmartDashboard::PutNumber("curr pose y meters", mDrive.getOdometryPose().Translation().Y().value());

        using namespace std::chrono_literals;

        // refresh rate of holonomic drive controller's PID controllers (edit if needed)
        double delayStart = frc::Timer::GetFPGATimestamp().value();
        while (mDrive.state == DriveState::Auto && frc::Timer::GetFPGATimestamp().value() - delayStart < 0.02) {
        };
    }
    mDrive.stopModules();
}

// EDIT LATER 
/**
 * Calls sequences of follow functions for set paths
 */
void Trajectory::followPath(Trajectory::autos autoTrajectory, bool flipAlliance) // true for red alliance
{
    switch (autoTrajectory)
    {
        case DO_NOTHING:
            break;
        case MOVE_STRAIGHT:
            follow ("Move Straight", flipAlliance, false, false);
            waitToScore(2);
            break;
        case auto_1A:
            //follow("Test Movement", flipAlliance, false, true, 0.0);
            follow("1 to A", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1B:
            follow("1 to B", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_1C:
            follow("1 to C", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1D:
            follow("1 to D", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1E:
            follow("1 to E", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_1F:
            follow("1 to F", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2A:
            follow("2 to A", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_2B:
            follow("2 to B", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_2C:
            follow("2 to C", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2D:
            follow("2 to D", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(2);
            follow("D to Bottom Coral Stationn", flipAlliance, false, false);
            break;
        case auto_2E:
            follow("2 to E", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_2F:    
            follow("2 to F", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3A:
            follow("3 to A", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to A", flipAlliance, false, false);
            waitToScore(2);
            follow("A to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_3B:
            follow("3 to B", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Top Coral Station to B", flipAlliance, false, false);
            waitToScore(2);
            follow("B to Top Coral Station", flipAlliance, false, false);
            break;
        case auto_3C:
            follow("3 to C", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to C", flipAlliance, false, false);
            waitToScore(2);
            follow("C to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3D:
            follow("3 to D", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to D", flipAlliance, false, false);
            waitToScore(2);
            follow("D to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3E:
            follow("3 to E", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to E", flipAlliance, false, false);
            waitToScore(2);
            follow("E to Bottom Coral Station", flipAlliance, false, false);
            break;
        case auto_3F:
            follow("3 to F", flipAlliance, false, true, 0.0);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            waitToScore(2);
            follow("Bottom Coral Station to F", flipAlliance, false, false);
            waitToScore(2);
            follow("F to Bottom Coral Station", flipAlliance, false, false);
            break;
    }
}

void Trajectory::waitToScore(int delaySeconds) {
    alignTimer.Start();
    mDrive.enableModules();

    while (!mAlign.isAligned(mLimelight) && alignTimer.Get() < 3_s) { // Timeout after 3 seconds
        ChassisSpeeds speeds = mAlign.autoAlign(mLimelight, 1, 0);
        mDrive.Drive(speeds, mGyro.getBoundedAngleCCW(), false);
        mDrive.updateOdometry();
    }
    mDrive.Drive(ChassisSpeeds(0, 0, 0), mGyro.getBoundedAngleCCW(), true, false);
    mDrive.stopModules();
    std::this_thread::sleep_for(std::chrono::seconds(delaySeconds));
}