#include "motor.hpp"

uint8_t pinStates[LENGTH][PIN_COUNT] = {
  //ENA,IN1,IN2,ENB,IN3,IN4
  {   1,  1,  0,  1,  1,  0}, // FORWARD
  {   1,  0,  1,  1,  0,  1}, // REVERSE
  {   1,  1,  1,  1,  1,  1}, // BRAKE
  {   0,  1,  0,  0,  1,  0}, // COAST
  {   1,  1,  0,  0,  1,  0}, // TURN_LEFT
  {   0,  1,  0,  1,  1,  0}, // TURN_RIGHT
  {   1,  1,  0,  1,  0,  1}, // PIVOT_LEFT
  {   1,  0,  1,  1,  1,  0}  // PIVOT_RIGHT
};

// Counter and distance 
volatile long counter = 0;

const float countsPerRotation = 886.0;
const float circumference = 0.1885;
volatile int totalDistance = 0;

volatile unsigned long lastChannelATime = 0;
volatile unsigned long lastChannelBTime = 0;
volatile unsigned long currChannelATime = 0;

volatile bool started = false;

// Non-interrupt functions
double getDistance() {
  return (double) counter / countsPerRotation * circumference;
}

// Interrupts
void readChannelA() {
 currChannelATime = millis();
 // Check whether B's pulse was closer to this pulse or the last one.
 // If it is closer to this pulse, we are moving from B to A
 // If it is closer to the last pulse, we are millis() - lastChannelBTime moving from A to B
 long d2 = lastChannelBTime - lastChannelATime;
 long d1 = currChannelATime - lastChannelBTime;
 if(started) {
  if(d1 > d2) {
    counter--;
  } else {
    counter++;
  }
 }

 lastChannelATime = currChannelATime;

 // Update stored distance value
 totalDistance = getDistance();

}

void readChannelB() {
 lastChannelBTime = millis();
}

// Class methods
Motor::Motor(
    uint8_t pin_ENA,
    uint8_t pin_IN1,
    uint8_t pin_IN2,
    uint8_t pin_ENB,
    uint8_t pin_IN3,
    uint8_t pin_IN4,
    uint8_t CHANNEL_A,
    uint8_t CHANNEL_B,
    uint8_t switch_count,
    uint8_t *switchPins
) {
    pinENA = pin_ENA;
    pinIN1 = pin_IN1;
    pinIN2 = pin_IN2;
    pinENB = pin_ENB;
    pinIN3 = pin_IN3;
    pinIN4 = pin_IN4;
    channelA = CHANNEL_A;
    channelB = CHANNEL_B;
    switchCount = switch_count;
    switchPins = switchPins;

    previousState = -1;
    currentState = BRAKE;

    currentSpeed = 0.1;
    previousSpeed = currentSpeed;

    name = "Motor";
}

/**
 * Sets the pins to the specified state
*/
void Motor::setPins(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t enB, uint8_t in3, uint8_t in4, int state) {
  if(state != BRAKE) {
    assert(in1 != in2);
    assert(in3 != in4);
  }

  if(DEBUG_ROBOT) Serial.print("Setting pins. ");
  if(DEBUG_ROBOT) Serial.print("State = ");
  if(DEBUG_ROBOT) Serial.println(currentState);

  /** LEFT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
  if(DEBUG_ROBOT) Serial.print("enA value: ");
  if(DEBUG_ROBOT) Serial.println(getPWMValue(enA));

  digitalWrite(pinIN1, in1);
  digitalWrite(pinIN2, in2);

  // Turn enable back on
  analogWrite(pinENA, getPWMValue(enA));

  /** RIGHT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
  if(DEBUG_ROBOT) Serial.print("enB value: ");
  if(DEBUG_ROBOT) Serial.println(getPWMValue(enB));

  digitalWrite(pinIN3, in3);
  digitalWrite(pinIN4, in4);

  // Turn enable back on
  analogWrite(pinENB, getPWMValue(enB));
}

void Motor::setAllPins() {
  setPins(pinStates[currentState][0],
            pinStates[currentState][1],
            pinStates[currentState][2],
            pinStates[currentState][3],
            pinStates[currentState][4],
            pinStates[currentState][5],
            currentState);
}

int Motor::getPWMValue(uint8_t isEnabled) {
  return (int) (255.0 * currentSpeed * isEnabled);
}

double Motor::getDistance() {
  return (double) counter / countsPerRotation * circumference;
}

void Motor::printDistance() {
  Serial.print("Distance travelled (cm): ");
  Serial.println(getDistance() * 100);

  Serial.print("Speed (cm/s): ");
  Serial.println(getDistance() * 100 / 2.0);

  if(DEBUG_ROBOT) Serial.print("\nCounter: ");
  if(DEBUG_ROBOT) Serial.println(counter);
}

void Motor::printPins() {
  Serial.print("Current pin states are: ");
  for(int i = 0; i < SWITCH_COUNT; i++) {
    Serial.print(digitalRead(switchPins[i]));
    Serial.print(", ");
  }
}

void Motor::startCounting() {
  started = true;
}

bool Motor::initialize() {
  // Configure outputs
  pinMode(pinENA, OUTPUT);
  pinMode(pinIN1, OUTPUT);
  pinMode(pinIN2, OUTPUT);
  pinMode(pinENB, OUTPUT);
  pinMode(pinIN3, OUTPUT);
  pinMode(pinIN4, OUTPUT);

  // Configure inputs for distance
  pinMode(channelA, INPUT);
  pinMode(channelB, INPUT);
  
  for(int i = 0; i < switchCount; i++) {
    pinMode(switchPins[i], INPUT);
  }

  // Configure interrupts
  attachInterrupt(channelA, readChannelA, CHANGE);
  attachInterrupt(channelB, readChannelB, CHANGE);
  
  return true;
}

bool Motor::systemsCheck() {
    move();
    forward();
    delay(100);
    reverse();
    delay(90);
    brake();
    return true;
}

void Motor::setSpeed(double speed) {
  currentSpeed = speed;
}

double Motor::getSpeed() {
    return currentSpeed;
}

void Motor::move() {
  // Set speed
  if(digitalRead(switchPins[0]) == HIGH)      currentSpeed = 1;
  else if(digitalRead(switchPins[1]) == HIGH) currentSpeed = 0.60;
  else if(digitalRead(switchPins[2]) == HIGH) currentSpeed = 0.40;
  else if(digitalRead(switchPins[3]) == HIGH) currentSpeed = 0.25;

  if(previousState != currentState || previousSpeed != currentSpeed) {
    if(DEBUG_ROBOT) {
        Serial.print("Changing state! Previous state: ");
        Serial.print(previousState);
        Serial.print(", Current State: ");
        Serial.println(currentState);
    }

    // Set pin values
    setAllPins();
  }

  previousState = currentState;
  previousSpeed = currentSpeed;
}
