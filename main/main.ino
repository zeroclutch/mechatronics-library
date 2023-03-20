#define DEBUG_ROBOT 0
#define PERFORM_SYSTEMS_CHECK 0

// Setting this to 0 will mean we only log when logger.dump() is called
#define DEBUG_TO_SERIAL 1

#include "logger.hpp"
#include "module.hpp"
#include "robot.hpp"

#define STARTING_STATE SeekLineState // actual
// #define STARTING_STATE SeekButtonState // testing only

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
  &LINE_PINS[0],
  EMITTER_PIN_REAR,
  &LINE_PINS_REAR[0]);

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
      if (action == "coinright") robot.setState(CoinRightState);
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
    if (distance.getDistance() > 34 && motor.getLeftDistance() > 1.6) {
      robot.setState(SeekCoinState);
    }


  } else if (robotState == SeekCoinState) {
    // Turn left
    motor.resetCounters();

    // Go straight 10cm
    while (motor.getLeftDistance() < 0.10) {
      float value = (float)lines.read();
      float difference = ((value - 3500) / 7000) * 0.4;

      // float difference = 0;
      float leftValue = 0.45 + difference;
      float rightValue = 0.45 - difference;

      motor.move();
    }

    motor.resetCounters();

    motor.setTargetSpeed(0, 0.4);
    motor.setSpeed(0, 0.4);
    while (lines.hasLine() || motor.getRightDistance() < 0.25) {
      motor.move();
    }

    motor.resetCounters();

    motor.setTargetSpeed(0, 0.4);
    motor.setSpeed(0, 0.4);
    while(!lines.hasLine() && motor.getRightDistance() < 0.1) {
      motor.move();
    }

    robot.setState(AlignCoinState);
  } else if (robotState == AlignCoinState) {
    // While we don't have a line, wiggle a bit for it.
    float difference = 0;
    float increment = 0.001;
    while (!lines.hasLine()) {
      float leftValue = 0.15 + difference; // typically we want the left slightly quicker
      float rightValue = 0.15 - difference;


      motor.setTargetSpeed(leftValue, rightValue);
      motor.setSpeed(leftValue, rightValue);

      motor.move();

      difference += increment;
      if(fabsf(difference) > 0.2) {
        increment *= -1;
      }
    }

    motor.resetCounters();
    while (!lines.hasCross()) {
      float value = (float)lines.read();
      float difference = ((value - 3500) / 7000) * 0.3;

      // float difference = 0;
      float leftValue = 0.35 + difference;
      float rightValue = 0.35 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }
    
    if(robot.coinsCollected < 2) {
      robot.setState(CoinLeftState);
    } else if(robot.coinsCollected < 4) {
      robot.setState(CoinRightState);
    } else {
      robot.setState(SeekButtonState);
    }

  } else if (robotState == CoinLeftState) {
    float pivotSpeed = 0.4;
    float idleSpeed = 0.05;
    float maxDistance = 0.15;


    // Pivot back right
    motor.resetCounters();
    motor.setTargetSpeed(-idleSpeed, -pivotSpeed);
    motor.setSpeed(-idleSpeed, -pivotSpeed);
    while(motor.getRightDistance() < maxDistance) {
      motor.move();
    }

    // Pivot back left
    motor.setSpeed(-pivotSpeed, -idleSpeed);
    motor.setTargetSpeed(-pivotSpeed, -idleSpeed);
    while(motor.getLeftDistance() < motor.getRightDistance()) {
      motor.move();
    }

    // Straight back
    motor.setSpeed(-pivotSpeed * 2, -pivotSpeed * 2);
    while(!digitalRead(REAR_BUMPER_PIN)) {
      motor.move();
    }

    robot.coinsCollected = 2;
    
    // Pivot front right
    motor.resetCounters();
    motor.setSpeed(pivotSpeed, idleSpeed);
    motor.setTargetSpeed(pivotSpeed, idleSpeed);
    while(motor.getLeftDistance() < maxDistance) {
      motor.move();
    }

    // Pivot front left
    motor.setSpeed(idleSpeed, pivotSpeed);
    motor.setTargetSpeed(idleSpeed, pivotSpeed);
    while(motor.getRightDistance() < motor.getLeftDistance()) {
      motor.move();
    }

    robot.setState(AlignCoinState);
  } else if (robotState == CoinRightState) {
    float pivotSpeed = 0.4;
    float idleSpeed = 0.05;
    float maxDistance = 0.24;

    // Pivot back left
    motor.resetCounters();
    motor.setSpeed(-pivotSpeed, -idleSpeed);
    motor.setTargetSpeed(-pivotSpeed, -idleSpeed);
    while(motor.getLeftDistance() < maxDistance) {
      motor.move();
    }

    // Pivot back right
    motor.setTargetSpeed(-idleSpeed, -pivotSpeed);
    motor.setSpeed(-idleSpeed, -pivotSpeed);
    while(motor.getRightDistance() < motor.getLeftDistance()) {
      motor.move();
    }

    motor.setSpeed(-pivotSpeed * 2, -pivotSpeed * 2);
    while(!digitalRead(REAR_BUMPER_PIN)) {
      motor.move();
    }

    robot.coinsCollected = 4;
    
    // Pivot front right
    motor.resetCounters();
    motor.setSpeed(idleSpeed, pivotSpeed);
    motor.setTargetSpeed(idleSpeed, pivotSpeed);
    while(motor.getRightDistance() < maxDistance) {
      motor.move();
    }

    // Pivot back right
    motor.setSpeed(pivotSpeed, idleSpeed);
    motor.setTargetSpeed(pivotSpeed, idleSpeed);
    while(motor.getLeftDistance() < motor.getRightDistance()) {
      motor.move();
    }

    robot.setState(AlignCoinState);
  } else if (robotState == SeekCrossState) {
    float value = (float) lines.read();
    
    float difference = ((value - 3500) / 7000) * 0.35;

    float leftValue = 0.25 + difference;
    float rightValue = 0.25 - difference;

    motor.setSpeed(leftValue, rightValue);
    motor.setTargetSpeed(leftValue, rightValue);

    motor.move();

    if (lines.hasCross()) {
      robot.setState(SeekButtonState);
    }
  } else if (robotState == SeekButtonState) {
    motor.resetCounters();
    lines.changePins(LINES_REAR);

    while(!lines.hasCross() && motor.getLeftDistance() < 0.10) {
      motor.setSpeed(0.35, 0.35);
      motor.move();
    }

    lines.changePins(LINES_FRONT);

    while(!digitalRead(FRONT_BUMPER_PIN)) {
      float value = (float) lines.read();
      float difference = ((value - 3500) / 7000) * 0.35;

      float leftValue = 0.35 + difference;
      float rightValue = 0.35 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(CenterRobotState);

  } else if (robotState == CenterRobotState) {
    motor.resetCounters();
    lines.changePins(LINES_REAR);
    // Find the center

    bool seenCross = false;
    float dist = robot.distanceFromCenter(robot.getPosition());
    while (!seenCross && motor.getLeftDistance() < dist) {
      float leftValue = -0.45;
      float rightValue = -0.45;
      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();

      seenCross = lines.hasCross() && seenCross;
    }

    robot.setState(SeekMoleState);
  } else if (robotState == SeekMoleState) {
    lines.changePins(LINES_FRONT);
    motor.resetCounters();

    // Pivot to correct position
    int curPos = robot.getPosition();
    int nextPos = robot.getTargetPosition();

    bool isRotatingRight = curPos < nextPos;

    float pivotSpeed = 0.30;

    if (isRotatingRight) {
      motor.setTargetSpeed(pivotSpeed, -pivotSpeed);
      motor.setSpeed(pivotSpeed, -pivotSpeed);
    } else if (!isRotatingRight) {
      motor.setTargetSpeed(-pivotSpeed, pivotSpeed);
      motor.setSpeed(-pivotSpeed, pivotSpeed);
    }

    float theta = 0.523598776;  // 2 * pi / 12
    float radius = motor.wheelbaseMeters / 2;
    float linearFactor = 0.95; // May need to increase
    float maxDistance = ((float) abs(curPos - nextPos)) * theta * radius * linearFactor;
    logger.log("I need to travel %d cm", (int) (maxDistance * 100));
    logger.log("I need to travel from %d to %d", curPos, nextPos);
    // Consider using line sensors for rotation instead

    uint16_t* sensorValues = lines.getSensorValues();
    bool reachedNewLine;

    // If we're already on a line, set it to true so we don't count an extra time
    if((isRotatingRight && sensorValues[6] > 2300) ||
      (!isRotatingRight && sensorValues[2] > 2300)) {
      reachedNewLine = true;
    } else {
      reachedNewLine = false;
    }

    bool isReady = curPos != nextPos;

    while(motor.getLeftDistance() < maxDistance || motor.getRightDistance() < maxDistance || isReady) {
      sensorValues = lines.getSensorValues();
      // Check limits
      if(isRotatingRight) {
        // logger.log("righto");
        if(!reachedNewLine && sensorValues[6] > 2300 && sensorValues[2] < 1000) {
          reachedNewLine = true;
          curPos++;
          logger.log("CURPOS:%d NEXTPOS:%d %u %u %u %u %u %u %u %u", curPos, nextPos, sensorValues[0], sensorValues[1], sensorValues[2], sensorValues[3], sensorValues[4], sensorValues[5], sensorValues[6], sensorValues[7]);
        }
        if(reachedNewLine && sensorValues[6] < 1000 && sensorValues[2] > 2300) {
          reachedNewLine = false;
        }
      }
      
      if(!isRotatingRight) {
        if(!reachedNewLine && sensorValues[6] < 1000 && sensorValues[2] > 2300) {
          reachedNewLine = true;
          curPos--;
        }
        if(reachedNewLine && sensorValues[6] > 2300 && sensorValues[2] < 1000) {
          reachedNewLine = false;
        }
      }
      motor.move();

      // Only complete when sensorValues[3] > 2300. We can also add a check for 4 to get better centering
      if(curPos == nextPos){// && (sensorValues[3] > 2300 || sensorValues[4] > 2300)) {
        isReady = false;
      }
    }

    // motor.brake();
    // unsigned long brakeStopTime = millis() + 500;
    // while(millis() < brakeStopTime) {
    //   motor.move();
    // }

    // sensorValues = lines.getSensorValues();


    motor.resetCounters();

    // // Pivot to line center
    // int readValue = 0;
    // while(abs(readValue - 3500) > 500) {
    //   readValue = lines.read();
    //   motor.move();
    // }
    
    robot.setPosition(nextPos);

    while (!digitalRead(FRONT_BUMPER_PIN)) {
      float value = (float)lines.read();
      float difference = ((value - 3500) / 7000) * 0.3;

      float leftValue = 0.4 + difference;
      float rightValue = 0.4 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);
      motor.move();
    }

    robot.setState(MoleColorState);
  } else if (robotState == MoleColorState) {
    delay(300); // Wait 150ms

    int color = tcsModule.getCurrent();
    logger.log("Current color: %d", color);
    if (color == TCS_RED) robot.setTargetPosition(MoleRed);
    else if (color == TCS_GREEN) robot.setTargetPosition(MoleGreen);
    else if (color == TCS_BLUE) robot.setTargetPosition(MoleBlue);
    else if (color == TCS_YELLOW) robot.setTargetPosition(MoleYellow);
    else if (color == TCS_PURPLE) robot.setTargetPosition(MolePurple);
    else if (color == TCS_WHITE) robot.setTargetPosition(MoleWhite);
    else robot.setTargetPosition(RedButton);  // uh oh

    // TODO: if we get the same color 3x in a row, we are probably wrong. Try an adjacent position

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
