#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef LINES_H
#define LINES_H

#include <QTRSensors.h>

#define LINES_FRONT 1
#define LINES_REAR 0

class Lines: public RobotModule {
    private:
        uint8_t emitterPin;
        uint8_t rearEmitterPin;
        const unsigned int calibrationMin = 500;
        const unsigned int calibrationMax = 2500;
        uint16_t lastValue = 0;

        const uint8_t *pins;
        const uint8_t *rearPins;

        bool isFront = true;

    public:
        Lines(uint8_t emitterPin, const uint8_t *pins, uint8_t rearEmitterPin, const uint8_t *rearPins);
        bool initialize();
        bool systemsCheck();
        
        bool isLineVisible = false;
        unsigned long lastPositionUpdate = 0;

        bool calibrate();

        bool hasLine();
        bool hasCross();
        uint16_t read();
        float getCurrentAngle();

        uint16_t* getSensorValues();

        void changePins(bool isFront);

};

#endif