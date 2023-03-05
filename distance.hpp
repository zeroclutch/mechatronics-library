#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef DISTANCE_H
#define DISTANCE_H

#define MAX_ANALOG_READ 613
#define DISTANCE_SENSOR_PIN A1
#define SAMPLE_COUNT 20


// Sorted list of input/output pairs
const float dataPoints[][2] = {
    {3.896,1},
    {3.665,2},
    {3.447,3},
    {3.242,4},
    {3.048,5},
    {2.867,6},
    {2.696,7},
    {2.537,8},
    {2.388,9},
    {2.249,10},
    {2.12,11},
    {1.999,12},
    {1.888,13},
    {1.785,14},
    {1.69,15},
    {1.603,16},
    {1.522,17},
    {1.449,18},
    {1.381,19},
    {1.32,20},
    {1.264,21},
    {1.213,22},
    {1.167,23},
    {1.125,24},
    {1.087,25},
    {1.052,26},
    {1.02,27},
    {0.991,28},
    {0.964,29},
    {0.939,30},
    {0.915,31},
    {0.892,32},
    {0.87,33},
    {0.847,34},
    {0.825,35},
    {0.801,36},
    {0.777,37},
    {0.751,38},
    {0.722,39},
    {0.692,40},
    {NULL, NULL}
};

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