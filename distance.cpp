#include "distance.hpp"

/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/

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
    {-1, -1}
};


/*
Measures the analog input voltage from the sensor.
b) Uses the information you gathered in the prior experiment to calculate the distance to
the wall. Use a piecewise linear model to approximate the conversion function.
c) The program should print the distance to the wall on the serial console each time
you press the pushbutton. Report the distance to the nearest 1mm when possible.
DEMO #1 – show the TA that this program works.
*/
Distance::Distance(int distanceSensorPin) {
    this->distanceSensorPin = distanceSensorPin;
    name = "Distance";
    outOfRange = false;
};

bool Distance::initialize() {
  // Configure pins
  pinMode(distanceSensorPin, INPUT);

  return true;
}

bool Distance::systemsCheck() {
  // Check if the sensor is working
  int value = readValue();
  float voltage = voltageFromValue(value);
  float distance = calculateDistance(voltage);
  logger->log("got distance.");

  if(distance < 0) {
    logger->log("Distance sensor is not working.");
    return false;
  }

  return true;
}

float Distance::calculateDistance(float voltage) {
  // Implement piecewise linear function
  int i = 0;
  while(dataPoints[i][0] > 0) {
    // See if next value is larger
    if(voltage > dataPoints[i + 1][0]) {
      // Calculate value
      float x1 = (float) dataPoints[i][0];
      float y1 = (float) dataPoints[i][1];
      float x0 = (float) dataPoints[i + 1][0];
      float y0 = (float) dataPoints[i + 1][1];

      // Point-slope form y = y0 + m(x - x0)
      return y0 + (y1-y0)/(x1-x0) * (voltage - x0);
    }
    i++;
  }
  return -1;
}

float Distance::voltageFromValue(int value) {
  if(value < 0) {
    return -1;
  }
  return value * 5.0 / 1023.0;
}

int Distance::readValue() {
  int value = analogRead(distanceSensorPin);
  return value;
}

void Distance::logSample(int value) {
  // Check if sample is valid
  samples[sampleIndex] = value;
  logger->log("[distance] Sample %d: %d", sampleIndex, value);
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
}

int Distance::averageSample() { 
  int sum = 0;
  int total = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    // If we don't have 20 samples yet, return the latest sample
    if(samples[i] == NULL) {
      break;
    }

    sum += samples[i];
    total++;
  }
  return sum/(total > 0 ? total : 1);
}

void Distance::updateDistance() {
    int value = readValue();
    logSample(value);
}

void Distance::logDistance() {
    int value = averageSample();
    float voltage = voltageFromValue(value);
    float distance = calculateDistance(voltage);

    logger->log("[distance] Raw value: %d", value);
    if(distance < 0) {
      logger->log("[distance] Distance is out of range.");
      return;
    } else {
      logger->log("[distance] The distance is approximately: %d mm.", (int) (distance * 10));
    }
}

float Distance::getDistance() {
  if(outOfRange) {
    return -1;
  } else {
    return calculateDistance(voltageFromValue(averageSample()));
  }
}
