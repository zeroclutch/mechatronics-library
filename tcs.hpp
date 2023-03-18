#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef _TCS34725_H_
#include "Adafruit_TCS34725.h"
#endif

#ifndef COLORS_H
#define COLORS_H


typedef struct TCSCOLOR {
    int r;
    int g;
    int b;
    int c;
} TCSColor;

enum TCS_COLOR_LIST {
    TCS_RED,
    TCS_GREEN,
    TCS_BLUE,
    TCS_WHITE,
    TCS_PURPLE,
    TCS_YELLOW,
    TCS_UNKNOWN
};

extern enum TCS_COLOR_LIST TCS_Color_List;


class TCS: public RobotModule {
    private:

        int discriminateByColor(int r, int g, int b, int c);
        int getDifference(TCSColor *a, TCSColor *b);

        int previousColor;
    public:
        TCS();
        Adafruit_TCS34725 *sensor;

        bool initialize();
        bool systemsCheck();

        int getCurrent();
        int getPreviousColor();

};

#endif