#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/config/SmartMotionConfig.h>
#include <rev/ClosedLoopSlot.h>

#define scoringMotorID 14
#define angleMotorID 15
#define angleMotor2ID 17

class EndEffector {

public:
    rev::spark::SparkMax scoringMotor = rev::spark::SparkMax {scoringMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless}; 
    rev::spark::SparkMaxConfig scoringConfig{};
    rev::spark::SparkClosedLoopController scoringCTR = scoringMotor.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder scoringEnc = scoringMotor.GetEncoder();

    rev::spark::SparkMax angleMotor1 = rev::spark::SparkMax {angleMotorID, rev::spark::SparkLowLevel::MotorType::kBrushless};
    rev::spark::SparkMax angleMotor2 = rev::spark::SparkMax {angleMotor2ID, rev::spark::SparkLowLevel::MotorType::kBrushless};

    rev::spark::SparkMaxConfig angle1Config{};
    rev::spark::SparkMaxConfig angle2Config{};

    rev::spark::SparkClosedLoopController angleCTR1 = angleMotor1.GetClosedLoopController();
    rev::spark::SparkClosedLoopController angleCTR2 = angleMotor2.GetClosedLoopController();
    rev::spark::SparkRelativeEncoder angleEnc1 = angleMotor1.GetEncoder();
    rev::spark::SparkRelativeEncoder angleEnc2 = angleMotor2.GetEncoder();

    double velocity = 5;
    double coralStation = 0;
    double scoreSetpoint = 0.02;

public:
    enum EndEffectorState {
        INTAKE,
        SCORE,
        AIM,
        STOP
    };
    EndEffectorState currentState = STOP;

    void init();
    void disable();
    void setState(EndEffectorState state);
    void intake();
    void aim();
    void score();
    void setVelocity(double speed);
};