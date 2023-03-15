#include "motor.hpp"

uint8_t pinStates[LENGTH][PIN_COUNT] = {
  //ENA,IN1,IN2,ENB,IN3,IN4
  {   1,  0,  1,  1,  0,  1}, // FORWARD
  {   1,  1,  0,  1,  1,  0}, // REVERSE
  {   1,  1,  1,  1,  1,  1}, // BRAKE
  {   0,  0,  1,  0,  0,  1}, // COAST
  {   1,  0,  1,  0,  0,  1}, // TURN_LEFT
  {   0,  0,  1,  1,  0,  1}, // TURN_RIGHT
  {   1,  0,  1,  1,  1,  0}, // PIVOT_LEFT
  {   1,  1,  0,  1,  0,  1}  // PIVOT_RIGHT
};

// Counter and distance 
volatile unsigned long leftCounter = 0;
volatile unsigned long rightCounter = 0;

const float countsPerRotation = 886 / 2;
const float circumference = 0.1885;

const int SPEED_CHECK_INTERVAL = 50000; // 5ms
volatile float lastLeftDistance = 0;
volatile float lastRightDistance = 0;
volatile float lastSpeedCheck = 0;

volatile float leftSpeed = 0;
volatile float rightSpeed = 0;

volatile float currentSpeedLeft = 0.0;
volatile float currentSpeedRight = 0.0;

volatile float previousSpeedRight = 0.0;
volatile float previousSpeedLeft = 0.0;

volatile float targetSpeedLeft = 0.0;
volatile float targetSpeedRight = 0.0;

volatile unsigned long lastChannelATime = 0;
volatile unsigned long lastChannelBTime = 0;
volatile unsigned long currChannelATime = 0;

volatile unsigned long lastChannelCTime = 0;
volatile unsigned long lastChannelDTime = 0;
volatile unsigned long currChannelCTime = 0;

volatile bool started = false;

// Non-interrupt functions
float getLeftDistance() {
  return (float) leftCounter / countsPerRotation * circumference;
}

float getRightDistance() {
  return (float) rightCounter / countsPerRotation * circumference;
}

// Interrupts
void readChannelA() {
  leftCounter++;
}

void readChannelB() {
  rightCounter++;
}
/*
// Interrupts
void readChannelC() {
  currChannelCTime = micros();
  // Check whether B's pulse was closer to this pulse or the last one.
  // If it is closer to this pulse, we are moving from B to A
  // If it is closer to the last pulse, we are millis() - lastChannelBTime moving from A to B
  long d2 = lastChannelDTime - lastChannelCTime;
  long d1 = currChannelCTime - lastChannelDTime;
  if(d1 > d2) {
    rightCounter--;
  } else {
    rightCounter++;
  }

  lastChannelCTime = currChannelCTime;
}

void readChannelD() {
 lastChannelDTime = micros();
}*/

bool is_zero(float speed) {
  return fabsf(speed) < 0.0001;
}

bool is_zero(float speed, float precision) {
  return fabsf(speed) < precision;
}


void updateLeftSpeed() {
  // Get the distance travelled since the last time we checked
  float newDistance = getLeftDistance();
  leftSpeed = (newDistance - lastLeftDistance) / (millis() - lastSpeedCheck) * 1000;
  lastLeftDistance = newDistance;
}

void updateRightSpeed() {
  // Get the distance travelled since the last time we checked
  float newDistance = getRightDistance();
  rightSpeed = (newDistance - lastRightDistance) / (millis() - lastSpeedCheck) * 1000;
  lastRightDistance = newDistance;
}

void updateSpeeds() {
  updateLeftSpeed();
  updateRightSpeed();

  /*
  lastSpeedCheck = millis();

  // Correct speed based on counter readings
  // We can assume that the trueSpeed and the currentSpeed are linearly related
  // This means that we can use the currentSpeed to calculate the trueSpeed
  // We can only do this if the trueSpeed has converged to the targetSpeed
  bool hasConverged = true;

  float trueRatio;
  float expectedRatio;

  if(is_zero(leftSpeed)) {
    trueRatio = -1;
  } else {
    trueRatio = rightSpeed / leftSpeed;
  }

  if(is_zero(targetSpeedLeft)) {
    expectedRatio = -1;
  } else {
    expectedRatio = targetSpeedRight / targetSpeedLeft;
  }

  float difference = (trueRatio - expectedRatio) / 2;

  if(expectedRatio < 0 || trueRatio < 0) {
    // We have not yet converged to the target speed
    hasConverged = false;
  }

  if(hasConverged) {
    if(is_zero(difference, 0.005)) {
    // If the true ratio is close enough to the expected ratio, do nothing
    } else  {
    // If right wheel is faster than expected, correct speeds
      float delta = targetSpeedLeft * difference;
      currentSpeedLeft += delta;
      currentSpeedRight -= delta;
    }
  }
  */
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
  uint8_t CHANNEL_C,
  uint8_t CHANNEL_D,
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
  channelC = CHANNEL_C;
  channelD = CHANNEL_D;
  switchCount = switch_count;
  switchPins = switchPins;

  previousState = BRAKE;
  currentState = BRAKE;

  currentSpeedRight = 0;
  currentSpeedLeft = 0;

  targetSpeedRight = 0;
  targetSpeedLeft = 0;

  previousSpeedLeft = 0;
  previousSpeedRight = 0;

  updatePreviousSpeed();

  name = "Motor";
}

Motor::~Motor() {
  // Nothing to do here
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
  pinMode(channelC, INPUT);
  pinMode(channelD, INPUT);
  
  for(int i = 0; i < switchCount; i++) {
    pinMode(switchPins[i], INPUT);
  }

  // Configure interrupts
  attachInterrupt(digitalPinToInterrupt(channelA), readChannelA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channelB), readChannelB, CHANGE);

  // Right wheel (check this)
  // attachInterrupt(digitalPinToInterrupt(channelC), readChannelC, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(channelD), readChannelD, CHANGE);
  
  Timer1.initialize(SPEED_CHECK_INTERVAL);
  Timer1.attachInterrupt(updateSpeeds);
  
  return true;
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

  logger->log("[motor] Setting motor pins. State=%d, enA=%d, enB=%d", currentState, enA, enB);

  /** LEFT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
  digitalWrite(pinIN1, in1);
  digitalWrite(pinIN2, in2);

  // Turn enable back on
  analogWrite(pinENA, getPWMValue(enA, WHEEL_LEFT));

  /** RIGHT WHEEL **/

  // Turn off enable momentarily so we don't accidentally brake during switching
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
    return (int) (255.0 * isEnabled * fabsf(currentSpeedLeft));
  } else if(wheel == WHEEL_RIGHT) {
    return (int) (255.0 * isEnabled * fabsf(currentSpeedRight));
  } else {
    logger->log("[motor] Invalid wheel value: %d", wheel);
    return 0;
  }
}


float Motor::getCurrentLeftSpeed() {
  return currentSpeedLeft;
};
float Motor::getCurrentRightSpeed() {
  return currentSpeedRight;
};

float Motor::getTargetLeftSpeed() {
  return targetSpeedLeft;
};
float Motor::getTargetRightSpeed() {
  return targetSpeedRight;
};

float Motor::getLeftDistance() {
  return ((float) leftCounter) / countsPerRotation * circumference;
}

float Motor::getRightDistance() {
  return ((float) rightCounter) / countsPerRotation * circumference;
}

void Motor::resetCounters() {
  leftCounter = 0;
  rightCounter = 0;
}

void Motor::printDistance() {
  logger->log("[motor] Distance travelled (cm): %f", getRightDistance() * 100);
  logger->log("[motor] Speed (cm/s): %f", getRightDistance() * 100 / 2.0);
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

bool Motor::systemsCheck() {

    setTargetSpeed(0.5, 0.5);
    logger->log("[motor] Speeds set to %d, %d", speedToInt(targetSpeedRight), speedToInt(targetSpeedLeft));
    move();
    if(!isZero(targetSpeedLeft - 0.5) || !isZero(targetSpeedRight - 0.5)) {
      logger->log("[motor] Speeds not set correctly.");
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
  return is_zero(speed);
}

bool Motor::isZero(float speed, float precision) {
  return is_zero(speed, precision);
}

void Motor::handleSpeedChange() {
  // If statement order is important to handle float precision issues
  if(isZero(currentSpeedRight)      && isZero(currentSpeedLeft)) coast();
  else if(isZero(currentSpeedRight) && currentSpeedLeft > 0)     turnLeft();
  else if(currentSpeedRight > 0     && isZero(currentSpeedLeft)) turnRight();
  else if(currentSpeedRight > 0     && currentSpeedLeft > 0)     forward();
  else if(currentSpeedRight < 0     && currentSpeedLeft < 0)     reverse();
  else if(currentSpeedRight < 0     && currentSpeedLeft > 0)     pivotLeft();
  else if(currentSpeedRight > 0     && currentSpeedLeft < 0)     pivotRight();
  else {
    logger->log("[motor] Invalid speed combination: %d, %d", speedToInt(currentSpeedRight), speedToInt(currentSpeedLeft));
    coast();
  }
}

void Motor::setSpeed(MotorSpeed* speed) {
  currentSpeedRight = clampSpeed(speed->right);
  currentSpeedLeft = clampSpeed(speed->left);
}

void Motor::setSpeed(float left, float right) {
  currentSpeedRight = clampSpeed(right);
  currentSpeedLeft = clampSpeed(left);
}

bool Motor::setTargetSpeed(MotorSpeed* speed) {
  return Motor::setTargetSpeed(speed->left, speed->right);
}

bool Motor::setTargetSpeed(float left, float right) {
  if(isZero(targetSpeedLeft - left) && isZero(targetSpeedRight - right)) {
    return false;
  }
  targetSpeedRight = clampSpeed(right);
  targetSpeedLeft = clampSpeed(left);

  logger->log("[motor] Target speeds set to %d, %d", speedToInt(targetSpeedRight), speedToInt(targetSpeedLeft));

  handleSpeedChange();
  return true;
}

// Returns the calculated speed in meters per second
float Motor::getTrueLeftSpeed() {
  return leftSpeed;
}

float Motor::getTrueRightSpeed() {
  return rightSpeed;
}

int Motor::speedToInt(float speed) {
  return (int) (speed * 100);
}

void Motor::updatePreviousSpeed() {
  previousSpeedRight = currentSpeedRight;
  previousSpeedLeft = currentSpeedLeft;
}

MotorSpeed* Motor::calculateSpeeds(MotorSpeed* dest, float averageSpeed, float angle) {
  dest->left = averageSpeed + angle;
  dest->right = averageSpeed - angle;
  return dest;
}

void Motor::followWall(float targetDistance, float currentDistance, float radius, float averageSpeed, int position) {
  // If error is positive, we are too close to the wall and need to turn more
  // If error is negative, we are too far from the wall and need to turn less
  // We travel 1/6 of the circumference of the circle each time we turn
  // Our new target speed should ensure that we maintain the same distance by the time we have turned 1/6 of the circle
  // This means that we need to turn less if we are too close to the wall and more if we are too far from the wall

  float error = targetDistance - currentDistance;
  //(1-p/6)e
  // Find a new radius based on the error and the distance to travel
  float newRadius = radius - error * averageSpeed * 10;

  float left = averageSpeed;
  float right = averageSpeed * (newRadius - wheelbaseCm) / newRadius; // TODO: compute this as constant 
  // logger->log("[motor] Error: %f, Angle: %f", error, angle);
  // Update the speeds based on the angle

  setTargetSpeed(left, right);
  setSpeed(left, right);
}

void Motor::move() {
  handleSpeedChange();
  bool stateOrSpeedChange = (
    previousState != currentState ||
    previousSpeedLeft != currentSpeedLeft ||
    previousSpeedRight != currentSpeedRight
  );

  if(stateOrSpeedChange) {
    logger->log("[motor] Changing state! Previous state: %d, Current state: %d", previousState, currentState); 
    // Set pin values
    setAllPins();
  }

  // logger->log("[motor] True ratio: %d, Expected ratio: %d, Difference: %d", (int) (trueRatio * 100), (int) (expectedRatio * 100), (int) (difference * 100));
  // logger->log("[motor] True left: %d, True right: %d", (int) (leftSpeed * 100), (int) (rightSpeed * 100));
  // logger->log("[motor] True speed: %d, %d", speedToInt(getTrueRightSpeed()), speedToInt(getTrueLeftSpeed()));
  // logger->log("[motor] Target speed: %d, %d", speedToInt(targetSpeedRight), speedToInt(targetSpeedLeft));
  

  // logger->log("[motor] Current distance: LEFT: %d, RIGHT %d", (int) (getLeftDistance() * 100), (int) (getRightDistance() * 100));
  /*
  // Update speed every SPEED_CHECK_INTERVAL - 1% every 2ms = 100% every 500ms
  if(micros() - lastSpeedUpdate > SPEED_UPDATE_INTERVAL) {
    logger->log("[motor] Interpolating speed after %luus", micros() - lastSpeedUpdate);
    // Update speed to target speed if not already there
    float rightDelta = targetSpeedRight - currentSpeedRight;
    if(!isZero(rightDelta)) currentSpeedRight += copysign(speedChangeRate, rightDelta);
    if(rightDelta < speedChangeRate) currentSpeedRight = targetSpeedRight;

    float leftDelta = targetSpeedLeft - currentSpeedLeft;
    if(!isZero(leftDelta)) currentSpeedLeft += copysign(speedChangeRate, leftDelta);
    if(leftDelta < speedChangeRate) currentSpeedLeft = targetSpeedLeft;
     
    // Update last speed update time
    lastSpeedUpdate = micros();
  }*/

  // currentSpeedLeft = targetSpeedLeft;
  // currentSpeedRight = targetSpeedRight;

  // Only update previous state after checking
  previousState = currentState;
  updatePreviousSpeed();
}
