#pragma once

#include <frc/DigitalInput.h>

class MagneticLimitSwitch {
    public:

        //constructor
        MagneticLimitSwitch(int pin) : magnetic_limit_switch(pin){}

        //checks if the switch is activated
        bool isActivated() {
            return magnetic_limit_switch.Get();
        }

    private:
        //private variable from DigitalInput library for the switch
        frc::DigitalInput magnetic_limit_switch;
};