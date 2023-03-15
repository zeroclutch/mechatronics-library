#include "tcs.hpp"

TCS::TCS() {
    this->sensor = Adafruit_TCS34725();
}

bool TCS::initialize() {
    tcsColorGuide[TCS_RED]    = { r: 6648, g: 770,   b: 877,   c: 8109 };  // 6648	770	877	8109
    tcsColorGuide[TCS_GREEN]  = { r: 2014, g: 7338,  b: 2935,  c: 13065 }; // 2014	7338	2935	13065
    tcsColorGuide[TCS_BLUE]   = { r: 1325, g: 3638,  b: 10506, c: 16062 }; // 1325	3638	10506	16062
    tcsColorGuide[TCS_YELLOW] = { r: 7865, g: 4708,  b: 2486,  c: 15264 }; // 7865	4708	2486	15264
    tcsColorGuide[TCS_WHITE]  = { r: 8455, g: 11309, b: 14663, c: 35343 }; // 8455	11309	14663	35343
    tcsColorGuide[TCS_PURPLE] = { r: 4831, g: 4338,  b: 13831, c: 23341 }; // 4831	4338	13831	23341

    // Potential issues:
    // 1. Sensor is not being read properly
    // 2. We may have to initialize sensor in the global scope
    return sensor.begin();
}

bool TCS::systemsCheck() {
    getCurrent();
    return true;
}

int TCS::getCurrent() {
  uint16_t r, g, b, c;

  sensor.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  //   colorTemp = sensor.calculateColorTemperature_dn40(r, g, b, c);
  //   lux = sensor.calculateLux(r, g, b);

  return discriminateByColor(r, g, b, c);
}

int TCS::discriminateByColor(int r, int g, int b, int c) {
    TCSColor currentColor = { r: r, g: g, b: b, c: c};
    int smallestDifference = 1000000;
    int smallestDifferenceIndex = 0;

    for(int i = 0; i < 6; i++) {
        int difference = getDifference(currentColor, tcsColorGuide[i]);

        if(difference < smallestDifference) {
            smallestDifference = difference;
            smallestDifferenceIndex = i;
        }
    }

    if(smallestDifference > 10000) {
        return TCS_UNKNOWN;
    }

    return smallestDifferenceIndex;
}

int TCS::getDifference(TCSColor a, TCSColor b) {
    return abs(a.r - b.r) + abs(a.g - b.g) + abs(a.b - b.b) + abs(a.c - b.c) ;
}
