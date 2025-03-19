#pragma once

#include <frc/Timer.h>
#include <util/ShuffleUI.h>
#include <frc/PowerDistribution.h>

#define swerveDriveStartCurrent 60
#define totalMatchSeconds 150

class PowerModule
{
private:
    bool reduceCurrentsOnBrownout;
    int currentIncrement = 5; // amps
    int timeToIncrement = 5; // seconds

public:
    frc::Timer updateTimer = frc::Timer();
    int driveCurrentLimit = swerveDriveStartCurrent;
    float minVoltage = 10.50;
    float maxVoltage = 13.0;

    frc::PowerDistribution mPDH = frc::PowerDistribution(1, frc::PowerDistribution::ModuleType::kRev);

    void init(bool enable)
    {
        reduceCurrentsOnBrownout = enable;
        updateTimer.Reset();
        updateTimer.Start();
    }
    int updateDriveCurrentLimit()
    {
        //ShuffleUI::MakeWidget("brownout", "Power", (int) mPDH.GetFaults().Brownout);
        if (reduceCurrentsOnBrownout)
        {
            if (mPDH.GetFaults().Brownout > 0)
            {
                driveCurrentLimit -= currentIncrement;
                mPDH.ClearStickyFaults();
                updateTimer.Reset();
            }
            else if (driveCurrentLimit < swerveDriveStartCurrent)
            {
                double timeSinceUpdate = updateTimer.Get().value();
                if (timeSinceUpdate > timeToIncrement)
                {
                    driveCurrentLimit += currentIncrement;
                    updateTimer.Reset();
                }
            }
        }
        //ShuffleUI::MakeWidget("driveCurrent", "Power", driveCurrentLimit);
        return driveCurrentLimit;
    }

    float currentScale() {
        frc::SmartDashboard::PutBoolean("Brownout?", mPDH.GetFaults().Brownout);
        if ((mPDH.GetFaults().Brownout == true) || (mPDH.GetVoltage() < minVoltage)) {
            float scale = (mPDH.GetVoltage() - minVoltage) / (maxVoltage - minVoltage);
            if (scale < minVoltage) {
                scale = 0.0;
            }
            return scale;
        }
        return 1.0;
    }
};
