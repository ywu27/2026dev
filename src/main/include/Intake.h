#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/SparkRelativeEncoder.h>
#include <string>
#include <util/ShuffleUI.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/config/SparkMaxConfig.h>
#include "sensors/ColorSensor.h"
#include "frc/Timer.h"

#define intakeID 17
#define angleID 16

class Intake
{
private:
    int intakeSpeed = 5700;
    bool timerStarted;
    const int clearCurrentThreshold = 20;
    const int clearVelocityThreshold = 1000;

public:

    rev::spark::SparkMax intakeMotor = rev::spark::SparkMax(intakeID, rev::spark::SparkMax::MotorType::kBrushless);
    rev::spark::SparkClosedLoopController intakeCtr = intakeMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder intakeEnc = intakeMotor.GetEncoder();
    rev::spark::SparkMaxConfig intakeConfig{};

    rev::spark::SparkMax angleMotor = rev::spark::SparkMax(angleID, rev::spark::SparkMax::MotorType::kBrushed);
    rev::spark::SparkClosedLoopController angleCtr = angleMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder angleEnc = angleMotor.GetEncoder();
    rev::spark::SparkMaxConfig angleConfig{};
    
    ColorSensor cSensor{frc::I2C::Port::kOnboard};
    enum intakeState
    {
        IN,
        CLEAR,
        HOLD,
        STOP
    };

    enum intakeAngle{
        UP,
        DOWN
    };

    // frc::Timer timer{};

    void init();
    void disable();
    void setState(intakeState state);
    void setSpeed(double speed);
    void clear();
    void hold();
    void setAngle(intakeAngle angle);
};