#pragma once

#include <rev/SparkMax.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkRelativeEncoder.h>
#include <frc/smartdashboard/SmartDashboard.h>

constexpr int ClimberID = 5;

class Climber{
    private:
        rev::spark::SparkMax climberMotor = rev::spark::SparkMax(ClimberID, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = climberMotor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController climberCTR = climberMotor.GetClosedLoopController();

        //Set velocity setpoint for climber
        double velocity;
    
    public:
        void init();
        void disable();
        void setVelocity(double speed);
        void velocitySetpoint();

};