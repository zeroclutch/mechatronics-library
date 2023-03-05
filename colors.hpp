#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef COLORS_H
#define COLORS_H

#define SAMPLE_COUNT 20
#define TIMEINTERVAL 10

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

typedef struct rgb {
  float r, g, b;
} RGB;

typedef struct hsl {
  float h, s, l;
} HSL;

class Colors: public RobotModule {
    private:
        int red_pin;
        int blue_pin;
        int green_pin;
        int white_pin;
        int photo_pin;

        int redReading[SAMPLE_COUNT];
        int greenReading[SAMPLE_COUNT];
        int blueReading[SAMPLE_COUNT];
        int whiteReading[SAMPLE_COUNT];

        bool debug;
        bool calibration_mode;

        void resolveColor(int color);
        void sampleLED(int LED, int reading[]);
        float getValues(int reading[]);
        int discriminateByRGB(float r, float g, float b, float w);
        bool firstIsBiggest(float a, float b, float c, float d, float e);
        HSL rgb2hsl(float r, float g, float b);
        
        int discriminateByHSL(float r, float g, float b, float w);


    public:
        Colors(
            int red_pin,
            int blue_pin,
            int green_pin,
            int white_pin,
            int photo_pin,
            bool debug,
            bool calibration_mode
        );

        bool initialize();
        bool systemsCheck();
        
        int getCurrent();
};

#endif