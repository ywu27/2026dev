#pragma once

#include <frc/controller/PIDController.h>
#include "Constants.h"

#include "photon/PhotonCamera.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/estimation/VisionEstimation.h"
#include "photon/PhotonPoseEstimator.h"
#include "photon/targeting/PhotonPipelineResult.h"

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

class PhotonVision {
private:
    double targetDistanceMeters;
    double targetYaw;
    double targetFiducial;
    double targetxMeters;
    
    double cameraPitchRadians;
    double cameraHeightMeters;
    double cameraPositionOffsetxMeters;
    double cameraPositionOffsetyMeters;

public:
    frc::AprilTagFieldLayout apriltagField = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
    photon::PhotonCamera camera;
    photon::PhotonPoseEstimator poseEstimator{
      apriltagField, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      frc::Transform3d(0.0_m, 0.0_m, 0.0_m, frc::Rotation3d(0.0_deg, 0.0_deg, 0.0_deg))};
    

    PhotonVision(std::string name)
        : camera(name),
          cameraPitchRadians(0.0),
          cameraHeightMeters(1.0), // FIX THIS
          cameraPositionOffsetxMeters(0.0),
          cameraPositionOffsetyMeters(0.0) {}

    void init() {
        camera.SetPipelineIndex(0);
    }

    void getInformationOfSpecificTargetFiducial(auto targetsSpan, int fiducial) {
        for (auto target : targetsSpan) {
            if (target.GetFiducialId() == fiducial) {
                targetDistanceMeters = target.GetBestCameraToTarget().Translation().Norm().value();
                targetYaw = target.GetBestCameraToTarget().Rotation().Z().value();
                targetFiducial = target.GetFiducialId();
                targetxMeters = target.GetBestCameraToTarget().Translation().X().value();
            }
        }
    }

    double getTargetx() {
        auto result = camera.GetLatestResult();
        photon::PhotonTrackedTarget target = result.GetBestTarget();
        return target.GetBestCameraToTarget().Translation().X().value();
    }

    double getTargety() {
        auto result = camera.GetLatestResult();
        photon::PhotonTrackedTarget target = result.GetBestTarget();
        return target.GetBestCameraToTarget().Translation().Y().value();
    }

    double getDistanceToTarget() {
        auto result = camera.GetLatestResult();
        photon::PhotonTrackedTarget target = result.GetBestTarget();
        return target.GetBestCameraToTarget().Translation().Norm().value();
    }

    double getYaw() {
        auto result = camera.GetLatestResult();
        photon::PhotonTrackedTarget target = result.GetBestTarget();
        return target.GetBestCameraToTarget().Rotation().Z().value() * 180.0 / M_PI;
    }

    frc::Pose2d returnPoseEstimate() {
        std::optional<photon::EstimatedRobotPose> visionCache;

        for (const auto& result : camera.GetAllUnreadResults()) {
            if (camera.GetLatestResult().HasTargets()) {
                visionCache = poseEstimator.Update(result);
                photon::PhotonPipelineResult latestResult = result;
            }
        }
        return visionCache->estimatedPose.ToPose2d();
    }
};