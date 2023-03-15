#include "lines.hpp"

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

Lines::Lines(uint8_t emitterPin, const uint8_t *pins) {
    name = "Lines";

    // configure the sensors
    qtr.setTypeRC();
    qtr.setSensorPins(pins, SensorCount);
    qtr.setEmitterPin(emitterPin);
}

bool Lines::initialize() {

    qtr.calibrate();

    // configure the calibration
    for(uint8_t i = 0; i < SensorCount; i++)
    {
        qtr.calibrationOn.minimum[i] =  500;
        qtr.calibrationOn.maximum[i] = 2500;
    }
    
    return true;
}

bool Lines::systemsCheck() {
    logger->log("[lines] Running systems check...");
    return true;
}

bool Lines::calibrate() {
    // analogRead() takes about 0.1 ms on an AVR.
  // 0.1 ms per sensor * 4 samples per sensor read (default) * 6 sensors
  // * 10 reads per calibrate() call = ~24 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 100; i++)
  {
    qtr.calibrate();
  }

  logger->log("[lines] Calibration complete.");

  // print the calibration minimum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  return true;
}

bool Lines::hasLine() {
  qtr.read(sensorValues);

  for(uint8_t i = 0; i < SensorCount; i++) {
    if(sensorValues[i] >= 2300) {
      lastPositionUpdate = millis() + 1000;
      return true;
    }
  }
  
  lastPositionUpdate = millis();
  return false;
}

bool Lines::hasCross() {
  qtr.read(sensorValues);
  
  int triggeredSensors = 0;
  for(uint8_t i = 0; i < SensorCount; i++) {
    if(sensorValues[i] >= 1500) {
      triggeredSensors++;
    }
  }

  logger->log("[lines] Triggered sensors: %d", triggeredSensors);
  
  // If all the sensors (except up to 2) are at max, we're on a cross
  return (triggeredSensors >= SensorCount - 2);
}

uint16_t Lines::read()
{
  // read raw sensor values
  uint16_t value = qtr.readLineBlack(sensorValues);

//     // read calibrated sensor values
//     qtr.read(sensorValues);

//     int sum = 0;
//     int divisor = 0;

//   for(int i = 0; i < SensorCount; i++) {
//     int reading = sensorValues[i];
//     sum += (1000 * i) * (reading - calibrationMin);
//     divisor += calibrationMax - calibrationMin;
//     logger->log("[lines] Sensor %d: %d", i, reading);
//   }

//   int value = sum / divisor;

  // print the sensor values as numbers from 0 to 1023, where 0 means maximum
  // reflectance and 1023 means minimum reflectance
  logger->log("[lines] The readValue is %d", value);

  lastValue = value;
  return value;
}


float Lines::getCurrentAngle() {
  // Expect all 8 sensors to have the same value
  // Calculate the angle based on the slope 
}