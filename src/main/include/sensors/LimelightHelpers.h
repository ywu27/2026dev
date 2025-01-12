#pragma once

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableValue.h"
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

namespace LimelightHelpers
{
    inline std::string sanitizeName(const std::string &name)
    {
        if (name == "")
        {
            return "limelight";
        }
        return name;
    }

    inline std::shared_ptr<nt::NetworkTable> getLimelightNTTable(const std::string &tableName)
    {
        return nt::NetworkTableInstance::GetDefault().GetTable(sanitizeName(tableName));
    }

    inline nt::NetworkTableEntry getLimelightNTTableEntry(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTable(tableName)->GetEntry(entryName);
    }

    inline double getLimelightNTDouble(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDouble(0.0);
    }

    inline std::vector<double> getLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetDoubleArray(std::span<double>{});
    }

    inline std::string getLimelightNTString(const std::string &tableName, const std::string &entryName)
    {
        return getLimelightNTTableEntry(tableName, entryName).GetString("");
    }

    inline void setLimelightNTDouble(const std::string &tableName, const std::string entryName, double val)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDouble(val);
    }

    inline void setLimelightNTDoubleArray(const std::string &tableName, const std::string &entryName, const std::span<const double> &vals)
    {
        getLimelightNTTableEntry(tableName, entryName).SetDoubleArray(vals);
    }

    inline double getTX(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tx");
    }

    inline double getTY(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ty");
    }

    inline double getTA(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "ta");
    }

    inline double getLatency_Pipeline(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tl");
    }

    inline double getLatency_Capture(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tl_cap");
    }

    inline std::string getJSONDump(const std::string &limelightName = "")
    {
        return getLimelightNTString(limelightName, "json");
    }

    inline std::vector<double> getBotpose(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose");
    }

    inline std::vector<double> getBotpose_wpiRed(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpired");
    }

    inline std::vector<double> getBotpose_wpiBlue(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_wpiblue");
    }

    inline std::vector<double> getBotpose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "botpose_targetSpace");
    }

    inline std::vector<double> getCameraPose_TargetSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "camerapose_targetspace");
    }

    inline std::vector<double> getTargetPose_CameraSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_cameraspace");
    }

    inline std::vector<double> getTargetPose_RobotSpace(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "targetpose_robotspace");
    }

    inline std::vector<double> getTargetColor(const std::string &limelightName = "")
    {
        return getLimelightNTDoubleArray(limelightName, "tc");
    }

    inline double getFiducialID(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tid");
    }

    inline double getNeuralClassID(const std::string &limelightName = "")
    {
        return getLimelightNTDouble(limelightName, "tclass");
    }

    inline void setPipelineIndex(const std::string &limelightName, int index)
    {
        setLimelightNTDouble(limelightName, "pipeline", index);
    }

    inline void setLEDMode_PipelineControl(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 0);
    }

    inline void setLEDMode_ForceOff(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 1);
    }

    inline void setLEDMode_ForceBlink(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 2);
    }

    inline void setLEDMode_ForceOn(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "ledMode", 3);
    }

    inline void setStreamMode_Standard(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 0);
    }

    inline void setStreamMode_PiPMain(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 1);
    }

    inline void setStreamMode_PiPSecondary(const std::string &limelightName)
    {
        setLimelightNTDouble(limelightName, "stream", 2);
    }

    inline const double INVALID_TARGET = 0.0;
    class SingleTargetingResultClass
    {
    public:
        SingleTargetingResultClass(){};
        ~SingleTargetingResultClass(){};
        double m_TargetXPixels{INVALID_TARGET};
        double m_TargetYPixels{INVALID_TARGET};

        double m_TargetXNormalized{INVALID_TARGET};
        double m_TargetYNormalized{INVALID_TARGET};

        double m_TargetXNormalizedCrosshairAdjusted{INVALID_TARGET};
        double m_TargetYNormalizedCrosshairAdjusted{INVALID_TARGET};

        double m_TargetXDegreesCrosshairAdjusted{INVALID_TARGET};
        double m_TargetYDegreesCrosshairAdjusted{INVALID_TARGET};

        double m_TargetAreaPixels{INVALID_TARGET};
        double m_TargetAreaNormalized{INVALID_TARGET};
        double m_TargetAreaNormalizedPercentage{INVALID_TARGET};

        // not included in json//
        double m_timeStamp{-1.0};
        double m_TargetLatency{0};
        double m_pipelineIndex{-1.0};
        std::vector<std::vector<double>> m_TargetCorners;

        std::vector<double> m_CAMERATransform6DTARGETSPACE;
        std::vector<double> m_TargetTransform6DCAMERASPACE;
        std::vector<double> m_TargetTransform6DROBOTSPACE;
        std::vector<double> m_ROBOTTransform6DTARGETSPACE;
        std::vector<double> m_ROBOTTransform6DFIELDSPACE;
    };

    class FiducialResultClass : public SingleTargetingResultClass
    {
    public:
        FiducialResultClass() {}
        ~FiducialResultClass() {}
        int m_fiducialID{0};
        std::string m_family{""};
    };

    class DetectionResultClass : public SingleTargetingResultClass
    {
    public:
        DetectionResultClass() {}
        ~DetectionResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class ClassificationResultClass : public SingleTargetingResultClass
    {
    public:
        ClassificationResultClass() {}
        ~ClassificationResultClass() {}

        int m_classID{-1};
        std::string m_className{""};
        double m_confidence{0};
    };

    class VisionResultsClass
    {
    public:
        VisionResultsClass() {}
        ~VisionResultsClass() {}
        std::vector<FiducialResultClass> FiducialResults;
        std::vector<DetectionResultClass> DetectionResults;
        std::vector<ClassificationResultClass> ClassificationResults;
        double m_timeStamp{-1.0};
        double m_TargetLatency{0};
        double m_JsonParseLatency{0};
        double m_pipelineIndex{-1.0};
        int valid{0};
        std::vector<double> botPose;
        std::vector<double> botPose_wpired;
        std::vector<double> botPose_wpiblue;
        void Clear()
        {
            FiducialResults.clear();
            DetectionResults.clear();
            ClassificationResults.clear();
            botPose.clear();
            botPose_wpired.clear();
            botPose_wpiblue.clear();
            m_timeStamp = -1.0;
            m_TargetLatency = 0;
            m_pipelineIndex = -1.0;
        }
    };

    class LimelightResultsClass
    {
    public:
        LimelightResultsClass() {}
        ~LimelightResultsClass() {}
        VisionResultsClass targetingResults;
    };

    namespace internal
    {
        inline const std::string _key_timestamp{"ts"};
        inline const std::string _key_latency{"tl"};
        inline const std::string _key_pipelineIndex{"pID"};
        inline const std::string _key_TargetXDegrees{"txdr"};
        inline const std::string _key_TargetYDegrees{"tydr"};
        inline const std::string _key_TargetXNormalized{"txnr"};
        inline const std::string _key_TargetYNormalized{"tynr"};
        inline const std::string _key_TargetXPixels{"txp"};
        inline const std::string _key_TargetYPixels{"typ"};

        inline const std::string _key_TargetXDegreesCrosshair{"tx"};
        inline const std::string _key_TargetYDegreesCrosshair{"ty"};
        inline const std::string _key_TargetXNormalizedCrosshair{"txn"};
        inline const std::string _key_TargetYNormalizedCrosshair{"tyn"};
        inline const std::string _key_TargetAreaNormalized{"ta"};
        inline const std::string _key_TargetAreaPixels{"tap"};
        inline const std::string _key_className{"class"};
        inline const std::string _key_classID{"classID"};
        inline const std::string _key_confidence{"conf"};
        inline const std::string _key_fiducialID{"fID"};
        inline const std::string _key_corners{"pts"};
        inline const std::string _key_transformCAMERAPOSE_TARGETSPACE{"t6c_ts"};
        inline const std::string _key_transformTARGETPOSE_CAMERASPACE{"t6t_cs"};
        inline const std::string _key_transformROBOTPOSE_TARGETSPACE{"t6r_ts"};
        inline const std::string _key_transformTARGETPOSE_ROBOTSPACE{"t6t_rs"};

        inline const std::string _key_transformROBOTPOSE_FIELDSPACE{"t6r_fs"};
        inline const std::string _key_skew{"skew"};
        inline const std::string _key_ffamily{"fam"};
        inline const std::string _key_colorRGB{"cRGB"};
        inline const std::string _key_colorHSV{"cHSV"};
    }
}