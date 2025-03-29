#pragma once

#include <frc/controller/PIDController.h>
#include "Constants.h"
#include "photon/PhotonCamera.h"
#include "photon/targeting/PhotonTrackedTarget.h"
#include "photon/estimation/VisionEstimation.h"
#include "photon/PhotonPoseEstimator.h"

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
    photon::PhotonCamera camera;

    PhotonVision(std::string name)
        : camera(name),
          cameraPitchRadians(0.0),
          cameraHeightMeters(1.0), // FIX THIS
          cameraPositionOffsetxMeters(0.0),
          cameraPositionOffsetyMeters(0.0) {}

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
};