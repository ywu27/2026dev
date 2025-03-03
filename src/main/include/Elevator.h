#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>

#define elevatorID1 11
#define elevatorID2 12

class Elevator {
    public:
        // enum elevatorState{
        //     START, LEVEL1, LEVEL2, LEVEL3, LEVEL4, CSTATION 
        // };
        int currentState = 0;

        void init();
        void setState(int state); // 0 = start, 1 = level 1, 2 = level 2, 3 = level 3, 4 = level 4, 5 = coral station
        void runMotor(double speed);
        void disable();
        int getCurrentState();

    private:
        //These setpoints need to be changed when the elevator is assembled...
        //The set points are relative. we subtracted 10 from each
        double starting_SP = 0.0;
        double level1 = 8.0;
        double level2 = 22.0; // rotations
        double cstation = 27.5;
        double level3 = 38.0;
        double level4 = 52.0;

        rev::spark::SparkMax motor = rev::spark::SparkMax(elevatorID1, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = motor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController elevatorCTR = motor.GetClosedLoopController();

        rev::spark::SparkMax motor2 = rev::spark::SparkMax(elevatorID2, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc2 = motor2.GetEncoder();
        rev::spark::SparkMaxConfig config2{};
        rev::spark::SparkClosedLoopController elevatorCTR2 = motor2.GetClosedLoopController();
};