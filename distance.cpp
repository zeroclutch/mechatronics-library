#include "distance.hpp"

/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/


/*
Measures the analog input voltage from the sensor.
b) Uses the information you gathered in the prior experiment to calculate the distance to
the wall. Use a piecewise linear model to approximate the conversion function.
c) The program should print the distance to the wall on the serial console each time
you press the pushbutton. Report the distance to the nearest 1mm when possible.
DEMO #1 – show the TA that this program works.
*/
Distance::Distance(int distanceSensorPin) {
    //this->distanceSensorPin = distanceSensorPin;
    name = "Distance";
};

bool Distance::initialize() {
  // Configure pins
  //pinMode(distanceSensorPin, INPUT);

  return true;
}

bool Distance::systemsCheck() {
  // Check if the sensor is working
  int value = readValue();
  float voltage = voltageFromValue(value);
  float distance = calculateDistance(voltage);

  if(distance < 0) {
    logger->log("Distance sensor is not working.");
    return false;
  }

  return true;
}

float Distance::calculateDistance(float voltage) {
  // Implement piecewise linear function
  int i = 0;
  while(dataPoints[i][0] != NULL) {
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
  }
  return -1;
}

float Distance::voltageFromValue(int value) {
  return value * 5.0 / 1023.0;
}

int Distance::readValue() {
  int value = analogRead(distanceSensorPin);
  return value;
}

void Distance::logSample(int value) {
  samples[sampleIndex] = value;
  sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;
}

int Distance::averageSample() { 
  int sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    sum += samples[i];

    // If we don't have 20 samples yet, return the latest sample
    if(samples[i] == NULL && i != 0) {
      return samples[i - 1];
    } else if (samples[i] == NULL) {
      return MAX_ANALOG_READ / 2;
    }
  }
  return sum/SAMPLE_COUNT;
}

void Distance::updateDistance() {
    int value = readValue();
    logSample(value);
}

void Distance::logDistance() {
    int value = averageSample();
    float voltage = voltageFromValue(value);

    logger->log("The distance is approximately: %f cm.", calculateDistance(voltage));
    logger->log("Raw value: %d, Voltage %f", value);
}

float Distance::getDistance() {
    return calculateDistance(averageSample());
}
