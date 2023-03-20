#include "tcs.hpp"

Adafruit_TCS34725 sensorObject = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_16X);

TCSColor tcsColorGuide[6];
TCS::TCS() {
    this->sensor = &sensorObject;

    // Original attempts
    // tcsColorGuide[TCS_RED]    = { r: 6648, g: 770,   b: 877,   c: 8109 };  // 6648	770	877	8109
    // tcsColorGuide[TCS_GREEN]  = { r: 2014, g: 7338,  b: 2935,  c: 13065 }; // 2014	7338	2935	13065
    // tcsColorGuide[TCS_BLUE]   = { r: 1325, g: 3638,  b: 10506, c: 16062 }; // 1325	3638	10506	16062
    // tcsColorGuide[TCS_WHITE]  = { r: 2537, g: 3223,  b: 4228,  c: 10180 }; // 8455	11309	14663	35343
    // tcsColorGuide[TCS_PURPLE] = { r: 4831, g: 4338,  b: 13831, c: 23341 }; // 4831	4338	13831	23341
    // tcsColorGuide[TCS_YELLOW] = { r: 7865, g: 4708,  b: 2486,  c: 15264 }; // 7865	4708	2486	15264
    

    // New attempts
    tcsColorGuide[TCS_RED]    = { r: 3302, g: 546,   b: 530,   c: 4333 };  // 3302, 546, 530, 4333
    tcsColorGuide[TCS_GREEN]  = { r: 991, g: 2499,  b: 1061,  c: 4843 }; // 991, 2499, 1061, 4843
    tcsColorGuide[TCS_BLUE]   = { r: 739, g: 1485,  b: 3832, c: 6265 }; // 739, 1485, 3832, 6265
    tcsColorGuide[TCS_WHITE]  = { r: 2704, g: 2872, b: 3680, c: 9326 }; // 2704, 2872, 3680, 9326
    tcsColorGuide[TCS_PURPLE] = { r: 1906, g: 1523,  b: 3904, c: 7480 }; // 1906, 1523, 3904, 7480
    tcsColorGuide[TCS_YELLOW] = { r: 2586, g: 1513,  b: 833,  c: 5026 }; // 2586, 1513, 833, 5026
}

bool TCS::initialize() {

    // Potential issues:
    // 1. Sensor is not being read properly
    // 2. We may have to initialize sensor in the global scope
    if(!sensor->begin()) {
        logger->log("TCS: Failed to initialize sensor");
        return false;
    } else {
        logger->log("TCS: Sensor initialized");
        return true;
    }
}

bool TCS::systemsCheck() {
    getCurrent();
    return true;
}

int TCS::getCurrent() {
  uint16_t r, g, b, c;

  sensor->getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
  //   colorTemp = sensor.calculateColorTemperature_dn40(r, g, b, c);
  //   lux = sensor.calculateLux(r, g, b);
  previousColor = discriminateByColor(r, g, b, c);
  logger->log("TCS: Current color is %d, %d, %d, %d", r, g, b, c);

  return previousColor;
}

int TCS::getPreviousColor() {
    return previousColor;
}

int TCS::discriminateByColor(int r, int g, int b, int c) {
    TCSColor currentColor = { r: r, g: g, b: b, c: c};
    int smallestDifference = 1000000;
    int smallestDifferenceIndex = TCS_UNKNOWN;

    for(int i = 0; i < 6; i++) {
        int difference = getDifference(&currentColor, &tcsColorGuide[i]);
        // logger->log("TCS: Difference is %d", difference);
        if(difference < smallestDifference) {
            smallestDifference = difference;
            smallestDifferenceIndex = i;
        }
    }

    if(smallestDifference > 25000) {
        return TCS_UNKNOWN;
    }

    return smallestDifferenceIndex;
}

int TCS::getDifference(TCSColor *a, TCSColor *b) {
    return (
        abs((a->r - b->r)) +
        abs((a->g - b->g)) +
        abs((a->b - b->b)) + 
        abs((a->c - b->c)) * 0.5
    );
}
