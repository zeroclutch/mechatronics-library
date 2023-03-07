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
    public:
        Lines(const uint8_t *pins);
        bool initialize();
        bool systemsCheck();

        bool calibrate();
        void read();
};

#endif