#ifndef COLORS_H
#include "colors.hpp"
#endif

/**
7. Add a red, green, blue, and white LED to your design (from your kit). Connect each one to an
Arduino digital output on one side and through a 330Ω resistor to ground on the other.
8. Create a discrimination function that determines if a paper sheet is:
a. Red
b. Green
c. Blue
d. Black
e. White
And turns on the corresponding LED to indicate which. (No LED for black)
**/
enum colorCodes {
  CODE_RED,
  CODE_BLUE,
  CODE_GREEN,
  CODE_WHITE,
  CODE_BLACK,

  // Unofficial colors
  CODE_ORANGE,
  CODE_YELLOW,
  CODE_PURPLE,
  CODE_PINK,
};

/******* CHANGE EVERYTHING BELOW *******/

// How to calibrate:
// FOR RGB:
//  1. Find the maximum R, G, B, W values for each color sheet.
//  2. Put those values into maxR, maxG, maxB, maxW, respectively.
// FOR HSL:
//  1. Approximate the sRGB value and put it into the calibrationRGB struct
//  2. Record the coefficients logged. Note those down.
//  3. Do this for every possible color sheet
//  4. Average the coefficients out and put them in rChange, gChange, bChange

// This is the RGB value we are estimating for a given object

RGB calibrationRGB = {
  r: 52.0,
  g: 89.0,
  b: 128.0
};

// GREEN: 65, 140, 76
// BLUE 52, 89, 128
// RED 230, 86, 76

// Coefficients (trueRgb/ourRgb)

float rChange = 1.10;
float gChange = 1.94;
float bChange = 1.16;

// srgb of object -> our rgb coordinates
// our rgb coordinates -> srgb -> hsl -> classify

const float maxR = 66.3;
const float maxG = 28.5;
const float maxB = 40.8;
const float maxW = 73.95;

/******* CHANGE EVERYTHING ABOVE *******/

Colors::Colors(int red, int green, int blue, int white, int photo, bool isDebug, bool isCalibration) {
  red_pin = red;
  green_pin = green;
  blue_pin = blue;
  white_pin = white;
  photo_pin = photo;
  debug = isDebug;
  calibration_mode = isCalibration;

  name = "Colors";
}


bool Colors::initialize() {
  // Configure pins
  pinMode(red_pin, OUTPUT);
  digitalWrite(red_pin, LOW);
  pinMode(blue_pin, OUTPUT);
  digitalWrite(blue_pin, LOW);
  pinMode(green_pin, OUTPUT);
  digitalWrite(green_pin, LOW);
  pinMode(white_pin, OUTPUT);
  digitalWrite(white_pin, LOW);

  analogReference(DEFAULT);
  
  pinMode(photo_pin, INPUT);

  return true;
}

bool Colors::systemsCheck() {
  // Check to see if the LEDs are working
  int iterations = 10;
  for(int i = iterations; i < 0; i--) {
    // Check to see if the LED is working
    digitalWrite(red_pin, HIGH);
    digitalWrite(green_pin, HIGH);
    digitalWrite(blue_pin, HIGH);
    digitalWrite(white_pin, HIGH);
    delay(50 * i);
    digitalWrite(red_pin, LOW);
    digitalWrite(green_pin, LOW);
    digitalWrite(blue_pin, LOW);
    digitalWrite(white_pin, LOW);
    delay(50 * i);
  }
  
  return true;
}

int Colors::getCurrent() {
  sampleLED(red_pin, redReading);
  delay(10);
  sampleLED(green_pin, greenReading);
  delay(10);
  sampleLED(blue_pin, blueReading);

  int r = getValues(redReading);
  int g = getValues(greenReading);
  int b = getValues(blueReading);
  int w = getValues(blueReading);

  if(debug) {
    Serial.print("r: ");
    Serial.println(r);
    Serial.print("g: ");
    Serial.println(g);
    Serial.print("b: ");
    Serial.println(b);
    Serial.print("w: ");
    Serial.println(w);
  }

  if(calibration_mode) {
    rChange = calibrationRGB.r / r;
    gChange = calibrationRGB.g / g;
    bChange = calibrationRGB.b / b;
    Serial.println("\n\nCalibration values:");
    Serial.print("rChange = ");
    Serial.print(rChange);
    Serial.print(";\ngChange = ");
    Serial.print(gChange);
    Serial.print(";\nbChange = ");
    Serial.print(bChange);
    Serial.println(";\n\n");
  }

  return discriminateByRGB(r, g, b, w);
}

void Colors::sampleLED(int LED, int reading[]) {
  if(debug) Serial.print("Sampling LED at pin ");
  if(debug) Serial.println(LED);

  reading[0] = analogRead(photo_pin);

  int i;

  // Take 100 samples of the LED
  for (i = 0; i < SAMPLE_COUNT; i++) {
    // Enable the LED
    digitalWrite(LED, HIGH);
    delayMicroseconds((unsigned long)(100 * TIMEINTERVAL));

    // Record the phototransistor reading
    reading[i] = analogRead(photo_pin);
    digitalWrite(LED, LOW);

    // Wait 100ms before taking another sample
    delay(100);
  }
}

float Colors::getValues(int reading[]) {
  int sum = 0;
  int i;
  for (i = 0; i < SAMPLE_COUNT; i++) {
    sum += reading[i];
  }
  return ((float) sum) / SAMPLE_COUNT / 1024 * 255.0;
}

void Colors::resolveColor(int color) {
  switch(color) {
    case CODE_RED: 
      Serial.println("Red!");
      break;
    case CODE_GREEN: 
      Serial.println("Green!");
      break;
    case CODE_BLUE: 
      Serial.println("Blue!");
      break;
    case CODE_WHITE: 
      Serial.println("White!");
      break;
    case CODE_BLACK: 
      Serial.println("Black!");
      break;
    case CODE_ORANGE: 
      Serial.println("Orange!");
      break;
    case CODE_YELLOW: 
      Serial.println("Black!");
      break;
    case CODE_PURPLE: 
      Serial.println("Black!");
      break;
    case CODE_PINK: 
      Serial.println("Black!");
      break;
    default:
      Serial.println("An unknown color was provided...");
  }
}

int Colors::discriminateByRGB(float r, float g, float b, float w) {
  // Normalize color 
  // Note, real normalization would also consider minR,
  // we are assuming that to be zero, but that may not
  // be the case.
  float percentR = ((float) r) / ((float) maxR);
  float percentG = ((float) g) / ((float) maxG);
  float percentB = ((float) b) / ((float) maxB);
  float percentW = ((float) w) / ((float) maxW);


  if(debug) Serial.print("percentR: ");
  if(debug) Serial.println(percentR);
  if(debug) Serial.print("percentG: ");
  if(debug) Serial.println(percentG);
  if(debug) Serial.print("percentB: ");
  if(debug) Serial.println(percentB);
  if(debug) Serial.print("percentW: ");
  if(debug) Serial.println(percentW);

  // If no color reaches 25% of its maximum, say it's black
  const float cutoffForBlack = 0.60;

  // Check black
  if(firstIsBiggest(cutoffForBlack, percentR, percentG, percentB, percentW)) {
    return CODE_BLACK;
  }

  if(firstIsBiggest(percentR, percentG, percentB, percentW, 0)) {
    return CODE_RED;
  }

  if(firstIsBiggest(percentG, percentB, percentW, 0, 0)) {
    return CODE_GREEN;
  }

  if(firstIsBiggest(percentB, percentW, 0, 0, 0)) {
    return CODE_BLUE;
  }

  return CODE_WHITE;

}

// Checks to see if the first argument has the largest value
bool Colors::firstIsBiggest(float a, float b, float c, float d, float e) {
  return a > b && a > c && a > d && a > e;
}

HSL Colors::rgb2hsl(float r, float g, float b) {
  
  HSL result;
  
  r /= 255;
  g /= 255;
  b /= 255;
  
  float max = MAX(MAX(r,g),b);
  float min = MIN(MIN(r,g),b);
  
  result.h = result.s = result.l = (max + min) / 2;

  if (max == min) {
    result.h = result.s = 0; // achromatic
  }
  else {
    float d = max - min;
    result.s = (result.l > 0.5) ? d / (2 - max - min) : d / (max + min);
    
    if (max == r) {
      result.h = (g - b) / d + (g < b ? 6 : 0);
    }
    else if (max == g) {
      result.h = (b - r) / d + 2;
    }
    else if (max == b) {
      result.h = (r - g) / d + 4;
    }
    
    result.h /= 6;
  }

  return result;
  
}

int Colors::discriminateByHSL(float r, float g, float b, float w) {
  // Multiply by coefficients
  HSL hslValue = rgb2hsl(r * rChange, g * gChange, b * bChange);
  
  // Filter out grays and whites
  if(w < 50) {
    return CODE_BLACK;// TODO: change these as needed
  }

  if(w > 220) {
    return CODE_WHITE; // TODO: change these as needed
  }
  
  float h = hslValue.h;
  // Get color
  if(h <= 0.04)  return CODE_RED;
  if(h <= 0.11)  return CODE_ORANGE;
  if(h <= 0.175) return CODE_YELLOW;
  if(h <= 0.46)  return CODE_GREEN;
  if(h <= 0.69)  return CODE_BLUE;
  if(h <= 0.77)  return CODE_PURPLE;
  if(h <= 0.96)  return CODE_PINK;
  return CODE_RED;
}
