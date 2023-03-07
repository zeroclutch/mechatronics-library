#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef LINES_H
#define LINES_H

#include "../QTRSensors/QTRSensors.h"
QTRSensors qtr;

const uint8_t SensorCount = 6;
uint16_t sensorValues[SensorCount];

class Lines: public RobotModule {
    private:
    public:
        Lines();
        bool initialize();
        bool systemsCheck();

        bool calibrateWhite();
        bool calibrateBlack();
};

#endif