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

    previousState = BRAKE;
    currentState = BRAKE;

    currentSpeed = (MotorSpeed*) malloc(sizeof(MotorSpeed));
    currentSpeed->right = 0;
    currentSpeed->left = 0;


    previousSpeed = (MotorSpeed*) malloc(sizeof(MotorSpeed));
    updatePreviousSpeed();

    name = "Motor";
}

Motor::~Motor() {
    free(previousSpeed);
    free(currentSpeed);
}

/**
 * Sets the pins to the specified state
*/
void Motor::setPins(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t enB, uint8_t in3, uint8_t in4, int state) {
  if(state != BRAKE) {
    if(in1 == in2 || in3 == in4) {
      logger->log("[motor] Error: braking outside of BRAKE state. State is %d.", state);
      logger->log("[motor] Error: in1 == in2: %d.", in1 == in2);
      logger->log("[motor] Error: in3 == in4: %d.", in3 == in4);
      return;
    }
  }

  logger->log("[motor] Setting motor pins. State = %d", currentState);

  /** LEFT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
  logger->log("[motor] enA value: %d", getPWMValue(enA, WHEEL_LEFT));

  digitalWrite(pinIN1, in1);
  digitalWrite(pinIN2, in2);

  // Turn enable back on
  analogWrite(pinENA, getPWMValue(enA, WHEEL_LEFT));

  /** RIGHT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
  logger->log("[motor] enB value: %d", getPWMValue(enB, WHEEL_RIGHT));

  digitalWrite(pinIN3, in3);
  digitalWrite(pinIN4, in4);

  // Turn enable back on
  analogWrite(pinENB, getPWMValue(enB, WHEEL_RIGHT));
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

int Motor::getPWMValue(uint8_t isEnabled, uint8_t wheel) {
  if(wheel == WHEEL_LEFT) {
    return (int) (255.0 * isEnabled * fabs(currentSpeed->left));
  } else if(wheel == WHEEL_RIGHT) {
    return (int) (255.0 * isEnabled * fabs(currentSpeed->right));
  } else {
    logger->log("[motor] Invalid wheel value: %d", wheel);
    return 0;
  }
}

double Motor::getDistance() {
  return (double) counter / countsPerRotation * circumference;
}

void Motor::printDistance() {
  logger->log("[motor] Distance travelled (cm): %f", getDistance() * 100);
  logger->log("[motor] Speed (cm/s): %f", getDistance() * 100 / 2.0);
  logger->log("[motor] Counter: %d", counter);
}

void Motor::printPins() {
  logger->log("[motor] Current pin states are: ");
  for(int i = 0; i < SWITCH_COUNT; i++) {
    logger->log("[motor] pin %d: %d", switchPins[i], digitalRead(switchPins[i]));
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
    setSpeed(0.5, 0.5);
    move();
    MotorSpeed* current = getSpeed();
    if(!isZero(currentSpeed->left - 0.5) || !isZero(currentSpeed->right - 0.5)) {
      logger->log("[motor] Speeds not set correctly.");
      return false;
    }

    logger->log("[motor] Speeds set to %d\%, %d\%", speedToInt(currentSpeed->right), speedToInt(currentSpeed->left));

    setSpeed(0.0, 0.0);
    move();
    current = getSpeed();
    if(!isZero(current->left) || !isZero(current->right)) {
      logger->log("[motor] Speeds not set correctly");
      return false;
    }


    return true;
}

float Motor::clampSpeed(float speed) {
  if(speed > 1.0) {
    return 1.0;
  } else if(speed < -1.0) {
    return -1.0;
  } 
  return speed;
}

bool Motor::isZero(float speed) {
  return fabs(speed) < 0.001;
}

bool Motor::isZero(float speed, float precision) {
  return fabs(speed) < precision;
}

void Motor::handleSpeedChange() {
  // If statement order is important to handle float precision issues
  if(isZero(currentSpeed->right)      && isZero(currentSpeed->left)) coast();
  else if(isZero(currentSpeed->right) && currentSpeed->left > 0)     turnLeft();
  else if(currentSpeed->right > 0     && isZero(currentSpeed->left)) turnRight();
  else if(currentSpeed->right > 0     && currentSpeed->left > 0)     forward();
  else if(currentSpeed->right < 0     && currentSpeed->left < 0)     reverse();
  else if(currentSpeed->right < 0     && currentSpeed->left > 0)     pivotLeft();
  else if(currentSpeed->right > 0     && currentSpeed->left < 0)     pivotRight();
  else {
    logger->log("[motor] Invalid speed combination: %d\%, %d\%", speedToInt(currentSpeed->right), speedToInt(currentSpeed->left));
    coast();
  }
}

void Motor::setSpeed(MotorSpeed* speed) {
  currentSpeed->right = clampSpeed(speed->right);
  currentSpeed->left = clampSpeed(speed->left);

  handleSpeedChange();
}

void Motor::setSpeed(float left, float right) {
  currentSpeed->right = clampSpeed(right);
  currentSpeed->left = clampSpeed(left);

  handleSpeedChange();
}

MotorSpeed* Motor::getSpeed() {
    return currentSpeed;
}

int Motor::speedToInt(float speed) {
  return (int) (speed * 100);
}

void Motor::updatePreviousSpeed() {
  previousSpeed->right = currentSpeed->right;
  previousSpeed->left = currentSpeed->left;
}

MotorSpeed* Motor::calculateSpeeds(MotorSpeed* dest, float averageSpeed, float angle) {
  dest->left = averageSpeed + angle;
  dest->right = averageSpeed - angle;
  return dest;
}

void Motor::move() {
  bool stateOrSpeedChange = (
    previousState != currentState ||
    previousSpeed->left != currentSpeed->left ||
    previousSpeed->right != currentSpeed->right
  );

  if(stateOrSpeedChange) {
    logger->log("[motor] Changing state! Previous state: %d, Current state: %d", previousState, currentState); 
    // Set pin values
    setAllPins();
  }

  // Only update previous state after checking
  previousState = currentState;
  updatePreviousSpeed();
}
