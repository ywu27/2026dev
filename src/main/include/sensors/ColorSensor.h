#pragma once
#include <rev/ColorSensorV3.h>
#include <rev/ColorMatch.h>
#include <frc/DigitalInput.h>
#include <frc/I2C.h>

class ColorSensor{
    private:
        rev::ColorSensorV3 colorSensor;
        rev::ColorMatch colorMatcher;
        static constexpr frc::Color algae = frc::Color(0.1545, 0.5432, 0.3024); 
    public:
        ColorSensor(frc::I2C::Port port) : colorSensor(port){}
        frc::Color getcolor(){
             frc::Color detectedColor = colorSensor.GetColor();
             return detectedColor;

        }

        bool isTarget(){
            frc::Color detectedColor = colorSensor.GetColor();
            bool red = (detectedColor.red < 0.16) && (detectedColor.red > 0.145);
            bool green = (detectedColor.green < 0.544) && (detectedColor.green > 0.529);
            bool blue = (detectedColor.blue < 0.33) && (detectedColor.blue > 0.29);
            if ((red && green && blue)){
                return true;
            }
            else {
                return false;
            }
        }
};