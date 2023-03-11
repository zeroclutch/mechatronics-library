#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef DISTANCE_H
#define DISTANCE_H

#define MAX_ANALOG_READ 613
#define SAMPLE_COUNT 20

class Distance: public RobotModule {
private:
    int distanceSensorPin;

    int samples[SAMPLE_COUNT];
    unsigned int sampleIndex = 0;

    int readValue();

    // Take a sample and add it to the sample array
    void logSample(int sample);
    int averageSample();

    float voltageFromValue(int value);
    float calculateDistance(float voltage);

  public:
    Distance(int DISTANCE_SENSOR_PIN);

    bool initialize();
    bool systemsCheck();

    void updateDistance();
    void logDistance();
    float getDistance();
};

#endif