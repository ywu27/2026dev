#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>
#include "sensors/ColorSensor.h"

#define intakeID 11
#define angleID 12

class Intake
{
private:
    int intakeSpeed = 5700;

    const int clearCurrentThreshold = 20;
    const int clearVelocityThreshold = 1000;

public:

    rev::spark::SparkMax intakeMotor = rev::spark::SparkMax(intakeID, rev::spark::SparkMax::MotorType::kBrushless);
    rev::spark::SparkClosedLoopController intakeCtr = intakeMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder intakeEnc = intakeMotor.GetEncoder();
    rev::spark::SparkMaxConfig intakeConfig{};

    rev::spark::SparkMax angleMotor = rev::spark::SparkMax(angleID, rev::spark::SparkMax::MotorType::kBrushless);
    rev::spark::SparkClosedLoopController angleCtr = angleMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder angleEnc = angleMotor.GetEncoder();
    rev::spark::SparkMaxConfig angleConfig{};
    
    ColorSensor cSensor{frc::I2C::Port::kOnboard};
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
    void clear();
};