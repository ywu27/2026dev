#pragma once

#include <rev/SparkMax.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkClosedLoopController.h>
#include <rev/config/SparkMaxConfig.h>

constexpr int ElevatorID = 3;//Update after it is finished


class Elevator {
    public:
        // Elevator heights
        enum elevatorState{
            START, LEVEL1, LEVEL2, LEVEL3, LEVEL4, CORALSTATION
        };
        elevatorState currentState = START;
        
        // Functions
        void init();
        void disable();
        void setState(elevatorState state);

    private:
        // The set points are relative. we subtracted 10 from each (in inches)
        double startLevel = 0.0; // lowest SP where the elevator starts
        double level1 = 8.0;
        double level2 = 22.0;
        double coralStation = 27.5;
        double level3 = 38.0;
        double level4 = 52.0; // highest SP for the L4 on reef

        // Initializations
        rev::spark::SparkMax motor = rev::spark::SparkMax(ElevatorID, rev::spark::SparkLowLevel::MotorType::kBrushless);
        rev::spark::SparkRelativeEncoder enc = motor.GetEncoder();
        rev::spark::SparkMaxConfig config{};
        rev::spark::SparkClosedLoopController elevatorCTR = motor.GetClosedLoopController();
        
};