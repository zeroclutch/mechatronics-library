#define DEBUG_ROBOT 1
#define PERFORM_SYSTEMS_CHECK 0

// Setting this to 0 will mean we only log when logger.dump() is called
#define DEBUG_TO_SERIAL 1

#include "logger.hpp"
#include "module.hpp"
#include "robot.hpp"

// #define STARTING_STATE SeekLineState
#define STARTING_STATE SeekButtonState

// Modules
#include "tcs.hpp"
#include "motor.hpp"
#include "distance.hpp"
#include "lines.hpp"

// Options
#define CALIBRATION_MODE 0
#define SWITCH_COUNT 4

/** DEFINE ALL PINS FOR USE **/

// Color sensor pins
#define PHOTOIN A0
#define WHITE_LED 40
#define GREEN_LED 42
#define RED_LED 44
#define BLUE_LED 46

// TCS color sensor pins
#define TCS_IN_1 SDA
#define TCS_IN_2 SCL

// Motor definitions
#define PIN_ENA 9
#define PIN_IN1 48
#define PIN_IN2 49
#define PIN_ENB 8
#define PIN_IN3 50
#define PIN_IN4 51
// Left wheel
#define CHANNEL_A 2
#define CHANNEL_B 3
// Right wheel
#define CHANNEL_C 18
#define CHANNEL_D 19
uint8_t switchPins[SWITCH_COUNT] = { 24, 25, 26, 27 };

// Lines
#define EMITTER_PIN 22
const uint8_t LINE_PINS[] = {
  23, 24, 25, 26, 27, 28, 29, 30
};

#define EMITTER_PIN_REAR A8
const uint8_t LINE_PINS_REAR[] = {
  A7, A6, A5, A4, A3, A2, A1, A0
};

// Distance definitions
#define DISTANCE_SENSOR_PIN A10

// Robot pins
#define START_PIN 4
#define REAR_BUMPER_PIN 46
#define FRONT_BUMPER_PIN 14
#define LED_PIN_COUNT 10
const uint8_t LED_PINS[10]{
  31, 32, 33, 34, 35, 36, 37, 38, 39, 40
};

/** END DEFINE PINS **/


// Instantiate modules
TCS tcsModule;

Motor motor(
  PIN_ENA,
  PIN_IN1,
  PIN_IN2,
  PIN_ENB,
  PIN_IN3,
  PIN_IN4,
  CHANNEL_A,
  CHANNEL_B,
  CHANNEL_C,
  CHANNEL_D,
  SWITCH_COUNT,
  *switchPins);

Distance distance(
  DISTANCE_SENSOR_PIN);

Lines lines(
  EMITTER_PIN,
  &LINE_PINS[0]);

Lines linesRear(
  EMITTER_PIN,
  &LINE_PINS[0]);

// Instantiate logger
Logger logger(
  DEBUG_ROBOT,
  DEBUG_TO_SERIAL);

// Instantiate robot
Robot robot(
  &LED_PINS[0],
  LED_PIN_COUNT);

void dump() {
  logger.dump();
}

void setup() {
  Serial.begin(115200);

  Serial.println("Starting...");
  setInterval(&dump, 1000);

  pinMode(REAR_BUMPER_PIN, INPUT_PULLUP);
  pinMode(FRONT_BUMPER_PIN, INPUT_PULLUP);
}

void loop() {
  readTimeouts();

  int robotState = robot.getState();

  // Setup robot. This should only occur once.
  if (robotState == InitializeState) {
    logger.log("[main] Initializing robot...");
    robot.attachLogger(&logger);

    // Load all modules
    robot.addModule(&tcsModule, TCSModule);
    robot.addModule(&motor, MotorModule);
    // robot.addModule(&linesRear, LinesRearModule);
    robot.addModule(&lines, LinesModule);
    robot.addModule(&distance, DistanceModule);

    // Setup robot
    bool isReady = robot.initialize();
    if (PERFORM_SYSTEMS_CHECK) {
      isReady = robot.systemsCheck() && isReady;
    }

    // Go to next states
    if (isReady) robot.setState(IdleState);
    else robot.setState(EndState);

  } else if (robotState == IdleState) {
    motor.setTargetSpeed(0, 0);
    motor.setSpeed(0, 0);
    motor.move();

    if (digitalRead(START_PIN)) {
      int delayCounter = 0;
      while (digitalRead(START_PIN)) {
        delay(1);
        delayCounter++;
        if (delayCounter > 100) {
          robot.setState(STARTING_STATE);
        }
      }
    }

    // Flush logger buffer
    if (Serial.available() != 0) {
      String action = Serial.readString();
      if (action == "seekline") robot.setState(SeekLineState);
      if (action == "followline") robot.setState(FollowLineState);
      if (action == "seekcoin") robot.setState(SeekCoinState);
      if (action == "aligncoin") robot.setState(AlignCoinState);
      if (action == "coinleft") robot.setState(CoinLeftState);
      if (action == "coinright") robot.setState(CoinLeftState);
      if (action == "seekcross") robot.setState(SeekCrossState);
      if (action == "seekbutton") robot.setState(SeekButtonState);
      if (action == "molecolor") robot.setState(MoleColorState);
      if (action == "seekmole") robot.setState(SeekMoleState);
      if (action == "centerrobot") robot.setState(CenterRobotState);
      if (action == "end") robot.setState(EndState);
    }

  } else if (robotState == SeekLineState) {
    motor.setTargetSpeed(0.5, 0.5);
    motor.setSpeed(0.5, 0.5);
    motor.move();

    if (lines.hasLine()) {
      robot.setState(FollowLineState);
    }

  } else if (robotState == FollowLineState) {
    float value = (float)lines.read();
    // int seed = millis() < 60000 ? millis() : 0;
    // float value = (float) (((int) seed) % 7000);
    float difference = ((value - 3500) / 7000) * 0.7;

    // float difference = 0;
    float leftValue = 0.35 + difference;
    float rightValue = 0.35 - difference;

    motor.setSpeed(leftValue, rightValue);
    motor.setTargetSpeed(leftValue, rightValue);

    motor.move();
    distance.updateDistance();
    if (distance.getDistance() > 34 && motor.getRightDistance() > 1.6) {
      robot.setState(SeekCoinState);
    }


  } else if (robotState == SeekCoinState) {
    // Turn left
    motor.resetCounters();
    while (lines.hasLine() || motor.getLeftDistance() < 0.25) {
      motor.setTargetSpeed(0.11, 0.23);
      motor.setSpeed(0.11, 0.23);

      motor.move();
    }



    while (!lines.hasLine()) {
      motor.move();
    }

    robot.setState(AlignCoinState);
  } else if (robotState == AlignCoinState) {
    motor.resetCounters();
    while (!lines.hasCross()) {
      float value = (float)lines.read();
      // int seed = millis() < 60000 ? millis() : 0;
      // float value = (float) (((int) seed) % 7000);
      float difference = ((value - 3500) / 7000) * 0.22;

      // float difference = 0;
      float leftValue = 0.17 + difference;
      float rightValue = 0.17 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }
    
    if(robot.coinsCollected < 3) {
      robot.setState(CoinLeftState);
    } else {
      robot.setState(CoinRightState);
    }

  } else if (robotState == CoinLeftState) {

    // Use PID with distance
    motor.resetCounters();
    float difference = 0.05;
    float leftValue = -0.2 + difference;
    float rightValue = -0.2 - difference;
    while(!digitalRead(REAR_BUMPER_PIN)) {
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.coinsCollected = 2;
    
    leftValue = 0.2 - difference;
    rightValue = 0.2 + difference;

    while(!lines.hasLine()) {
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(AlignCoinState);
  } else if (robotState == CoinRightState) {

    // Use PID with distance
    motor.resetCounters();
    float difference = 0.02;
    float leftValue = -0.2 + difference;
    float rightValue = -0.2 - difference;
    while(!digitalRead(REAR_BUMPER_PIN)) {
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.coinsCollected = 4;
    
    leftValue = 0.2 - difference;
    rightValue = 0.2 + difference;

    while(!lines.hasLine()) {
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(SeekCrossState);
  } else if (robotState == SeekCrossState) {
    float value = (float)lines.read();
    
    float difference = ((value - 3500) / 7000) * 0.2;

    float leftValue = 0.3 + difference;
    float rightValue = 0.3 - difference;

    motor.setSpeed(leftValue, rightValue);
    motor.setTargetSpeed(leftValue, rightValue);

    motor.move();

    if (lines.hasCross()) {
      robot.setState(SeekButtonState);
    }
  } else if (robotState == SeekButtonState) {
    motor.resetCounters();
    while(!linesRear.hasCross() && motor.getLeftDistance() < 0.10) {
      motor.setSpeed(0.3, 0.3);
      motor.move();
    }

    while(!digitalRead(FRONT_BUMPER_PIN)) {
      float value = (float) lines.read();
      float difference = ((value - 3500) / 7000) * 0.2;

      float leftValue = 0.3 + difference;
      float rightValue = 0.3 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(CenterRobotState);

  } else if (robotState == CenterRobotState) {
    motor.resetCounters();
    // Find the center
    while (!linesRear.hasCross()) {
      float value = (float) linesRear.read();
      float difference = ((value - 3500) / 7000) * 0.2;

      // float difference = 0;
      float leftValue = -0.2 + difference;
      float rightValue = -0.2 - difference;
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(SeekMoleState);
  } else if (robotState == SeekMoleState) {

    // Pivot to correct position
    int curPos = robot.getPosition();
    int nextPos = robot.getTargetPosition();
    if (curPos > nextPos) {
      motor.setTargetSpeed(0.2, -0.2);
      motor.setSpeed(0.2, -0.2);
    } else if (curPos < nextPos) {
      motor.setTargetSpeed(-0.2, 0.2);
      motor.setSpeed(-0.2, 0.2);
    }

    float theta = 0.523;  // 2 * pi / 12
    float radius = motor.wheelbaseMeters / 2;
    float linearFactor = 1;
    float distance = ((float) abs(curPos - nextPos)) * theta * radius ;
    logger.log("I need to travel %d cm", (int) (distance * 100));
    logger.log("I need to travel from %d to %d", curPos, nextPos);
    // Consider using line sensors for rotation instead

    motor.resetCounters();
    while (motor.getLeftDistance() < distance) {
      logger.log("Distance travelled: %d", (int) (motor.getLeftDistance() * 100));
      motor.move();
    }

    // Alternative: turn until we lose a line
    // continue until we get 3500~ line reading 
    // increment or decrement pos
    // rinse/repeat until we are at the right one

    robot.setPosition(nextPos);

    while (!digitalRead(FRONT_BUMPER_PIN)) {
      float value = (float)lines.read();
      float difference = ((value - 3500) / 7000) * 0.2;

      float leftValue = 0.35 + difference;
      float rightValue = 0.35 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);
      motor.move();
    }

    robot.setState(MoleColorState);
  } else if (robotState == MoleColorState) {
    int color = tcsModule.getCurrent();
    logger.log("Current color: %d", color);
    if (color == TCS_RED) robot.setTargetPosition(MoleRed);
    else if (color == TCS_GREEN) robot.setTargetPosition(MoleGreen);
    else if (color == TCS_BLUE) robot.setTargetPosition(MoleBlue);
    else if (color == TCS_YELLOW) robot.setTargetPosition(MoleYellow);
    else if (color == TCS_PURPLE) robot.setTargetPosition(MolePurple);
    else if (color == TCS_WHITE) robot.setTargetPosition(MoleWhite);
    else robot.setTargetPosition(MoleWhite);  // uh oh

    robot.setState(CenterRobotState);
  } else {
    Serial.println("end/default state");
    logger.dump();
    exit(0);
  }

  if (Serial.available() != 0) {
    String action = Serial.readString();
    if (action == "stop") robot.setState(IdleState);
  }
}
