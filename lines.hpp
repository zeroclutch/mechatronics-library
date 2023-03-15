#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef LINES_H
#define LINES_H

#include <QTRSensors.h>

class Lines: public RobotModule {
    private:
        uint8_t emitterPin;
        const unsigned int calibrationMin = 500;
        const unsigned int calibrationMax = 2500;
        uint16_t lastValue = 0;

    public:
        Lines(uint8_t emitterPin, const uint8_t *pins);
        bool initialize();
        bool systemsCheck();
        
        bool isLineVisible = false;
        unsigned long lastPositionUpdate = 0;

        bool calibrate();

        bool hasLine();
        bool hasCross();
        uint16_t read();
        float getCurrentAngle();

};

#endif