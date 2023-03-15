#define DEBUG_ROBOT 1
#define PERFORM_SYSTEMS_CHECK 0

// Setting this to 0 will mean we only log when logger.dump() is called
#define DEBUG_TO_SERIAL 0

#include "logger.hpp"
#include "module.hpp"
#include "robot.hpp"

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
#define PHOTOIN      A0
#define WHITE_LED    40
#define GREEN_LED    42
#define RED_LED      44
#define BLUE_LED     46

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
uint8_t switchPins[SWITCH_COUNT] = {24, 25, 26, 27};

// Lines
#define EMITTER_PIN 22
const uint8_t LINE_PINS[] = {
  23, 24, 25, 26, 27, 28, 29, 30
};

// Distance definitions
#define DISTANCE_SENSOR_PIN A10

// Robot pins
#define START_PIN 4
#define REAR_BUMPER_PIN 46
#define FRONT_BUMPER_PIN 47
#define LED_PIN_COUNT 10
const uint8_t LED_PINS[10] {
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
  *switchPins
);

Distance distance(
  DISTANCE_SENSOR_PIN
);

Lines lines(
  EMITTER_PIN,
  &LINE_PINS[0]
);

// Instantiate logger
Logger logger(
  DEBUG_ROBOT,
  DEBUG_TO_SERIAL);

// Instantiate robot
Robot robot(
  &LED_PINS[0],
  LED_PIN_COUNT
);

void dump() {logger.dump();}

void setup() {
  Serial.begin(115200);

  Serial.println("Starting...");
  setInterval(&dump, 1000);
}

void loop() {
  readTimeouts();

  int robotState = robot.getState();

  // Setup robot. This should only occur once.
  if(robotState ==  InitializeState) {
    logger.log("[main] Initializing robot...");
    robot.attachLogger(&logger);

    // Load all modules
    robot.addModule(&tcsModule, TCSModule);
    robot.addModule(&motor, MotorModule);
    robot.addModule(&lines, LinesModule);
    robot.addModule(&distance, DistanceModule);

    // Setup robot
    bool isReady = robot.initialize();
    if(PERFORM_SYSTEMS_CHECK) {
      isReady = robot.systemsCheck() && isReady;
    }

    // Go to next states
    if(isReady) robot.setState(IdleState);
    else robot.setState(EndState);

  } else if(robotState ==  IdleState) {
    motor.setTargetSpeed(0,0);
    motor.setSpeed(0,0);
    motor.move();

    if(digitalRead(START_PIN)) { 
      int delayCounter = 0;
      while(digitalRead(START_PIN)) {
        delay(1);
        delayCounter++;
        if(delayCounter > 100) {
          robot.setState(SeekLineState);
        }
      }
    }
    
    // Flush logger buffer
    if(Serial.available() != 0) {
      String action = Serial.readString();
      if(action == "go") robot.setState(FollowLineState);
      if(action == "seek") robot.setState(SeekLineState);
      if(action == "seekcoin") robot.setState(SeekCoinState);
      if(action == "calibrate") robot.setState(CalibrateState);
      if(action == "mole") {
        robot.setState(SeekMoleState);
      }
    }

  } else if (robotState ==  CalibrateState) {
    lines.calibrate();
    robot.setState(IdleState);
  } else if (robotState ==  SeekLineState) {
    motor.setTargetSpeed(0.5,0.5);
    motor.setSpeed(0.5,0.5);
    motor.move();

    if(lines.hasLine()) {
      robot.setState(FollowLineState);
    }
    
  } else if (robotState ==  FollowLineState) {
    float value = (float) lines.read();
    // int seed = millis() < 60000 ? millis() : 0;
    // float value = (float) (((int) seed) % 7000);
    float difference = ((value - 3500) / 7000) * 0.7;

    // float difference = 0;
    float leftValue =  0.35 + difference;
    float rightValue = 0.35 - difference;

    motor.setSpeed(leftValue, rightValue);
    motor.setTargetSpeed(leftValue, rightValue);

    motor.move();
    distance.updateDistance();
    if(distance.getDistance() > 32) {
      robot.setState(SeekCoinState);
    }


  } else if (robotState ==  SeekCoinState) {
    // Turn left
    motor.resetCounters();
    while(lines.hasLine() || motor.getLeftDistance() < 1) {
      motor.setTargetSpeed(0.11, 0.23);
      motor.setSpeed(0.11, 0.23);

      motor.move();
    }



    while(!lines.hasLine()) {
      distance.updateDistance();
      float value = distance.getDistance();
      float difference = ((value - 15) / value) * 0.05;

      float leftValue =  0.1 - difference;
      float rightValue = 0.1 + difference;
      motor.setTargetSpeed(leftValue, rightValue);
      motor.setSpeed(leftValue, rightValue);

      motor.move();
      break;
    }

    robot.setState(AlignCoinState);
  } else if (robotState ==  AlignCoinState) {
    motor.resetCounters();
    while(!lines.hasCross()) { 
      float value = (float) lines.read();
      // int seed = millis() < 60000 ? millis() : 0;
      // float value = (float) (((int) seed) % 7000);
      float difference = ((value - 3500) / 7000) * 0.2;

      // float difference = 0;
      float leftValue =  0.2 + difference;
      float rightValue = 0.2 - difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    robot.setState(CoinState);
    
  } else if (robotState ==  CoinState) {

    motor.setSpeed(-0.2, -0.2);
    motor.setTargetSpeed(-0.2, -0.2);

    motor.move();

    while(lines.hasLine()) { // 10cm
      float value = (float) lines.read();
      // int seed = millis() < 60000 ? millis() : 0;
      // float value = (float) (((int) seed) % 7000);
      float difference = ((value - 3500) / 7000) * 0.1;

      // float difference = 0;
      float leftValue =  -0.2 - difference;
      float rightValue = -0.2 + difference;

      motor.setSpeed(leftValue, rightValue);
      motor.setTargetSpeed(leftValue, rightValue);

      motor.move();
    }

    if(digitalRead(REAR_BUMPER_PIN)) {
      robot.setState(SeekCrossState);
    }
  } else if (robotState ==  SeekCrossState) {
    float value = (float) lines.read();
    if(!lines.hasLine()) {
      value = 3500;
    }
    float difference = ((value - 3500) / 7000) * 0.2;

    float leftValue =  0.3 + difference;
    float rightValue = 0.3 - difference;

    motor.setSpeed(leftValue, rightValue);
    motor.setTargetSpeed(leftValue, rightValue);

    motor.move();

    if(lines.hasCross()) {
      robot.setState(SeekButtonState);
    }
  } else if (robotState ==  SeekButtonState) {
    if(digitalRead(FRONT_BUMPER_PIN)) {
      robot.setState(IdleState);
    } else {
      motor.setSpeed(0.3, 0.3);
      motor.move();
    }
  } else if (robotState ==  SeekMoleState) {
    bool isFirstIteration = motor.setTargetSpeed(0.5, 0.3);
    if(isFirstIteration) {
      motor.setSpeed(0.5, 0.3);
    }

    logger.log("Left current: %d, Right current: %d, Left target: %d, Right target: %d",
    (int) (motor.getCurrentLeftSpeed() * 100),
    (int) (motor.getCurrentRightSpeed() * 100),
    (int) (motor.getTargetLeftSpeed()  * 100),
    (int) (motor.getTargetRightSpeed() * 100));

    logger.log("Left true: %d, Right true: %d",
    (int) (motor.getTrueLeftSpeed() * 100),
    (int) (motor.getTrueRightSpeed() * 100));

    motor.move();
  } else {
    Serial.println("end/default state");
    logger.dump();
    exit(0);
  }

  if(Serial.available() != 0) {
    String action = Serial.readString();
    if(action == "stop") robot.setState(IdleState);
  }

}


