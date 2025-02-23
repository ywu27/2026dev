#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>

int intakeMotorID= 1;
int clearingCurrentLimit=40;
int intakeCurrentLimit=40;

class Intake
{
private:
    int intakeSpeed = 5700;

    const int clearCurrentThreshold = 40;
    const int clearVelocityThreshold = 100;

    rev::spark::SparkMax intakeMotor = rev::spark::SparkMax(intakeMotorID, rev::spark::SparkMax::MotorType::kBrushless);
    rev::spark::SparkClosedLoopController intakeController = intakeMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder intakeEncoder = intakeMotor.GetEncoder();
    rev::spark::SparkMaxConfig config{};
    void clear();

public:
    enum intakeState
    {
        IN,
        CLEAR,
        STOP
    };

    void init();
    void disable();
    void setIntakeState(intakeState state);
    void setIntakeSpeed(double speed);
};