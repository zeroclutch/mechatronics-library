#define DEBUG_ROBOT 1
#define PERFORM_SYSTEMS_CHECK 1

// Setting this to 0 will mean we only log when logger.dump() is called
#define DEBUG_TO_SERIAL 1

#include "logger.hpp"
#include "module.hpp"
#include "robot.hpp"

// Modules
#include "colors.hpp"
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

/** END DEFINE PINS **/


// Instantiate modules
Colors colors(
  RED_LED,
  BLUE_LED,
  GREEN_LED,
  WHITE_LED,
  PHOTOIN,
  DEBUG_ROBOT,
  CALIBRATION_MODE
);

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
Logger logger(DEBUG_ROBOT, DEBUG_TO_SERIAL);

// Instantiate robot
Robot robot;

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
    robot.addModule(&colors, ColorsModule);
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

    // robot.setState(FollowLineState);
    
    // Flush logger buffer
    if(Serial.available() != 0) {
      String action = Serial.readString();
      if(action == "go") robot.setState(FollowLineState);
      if(action == "seek") robot.setState(SeekLineState);
      if(action == "calibrate") robot.setState(CalibrateState);
      if(action == "mole") robot.setState(MoleState);
    }

  } else if (robotState ==  CalibrateState) {
    lines.calibrate();
    robot.setState(IdleState);
  } else if (robotState ==  SeekLineState) {
    motor.setTargetSpeed(1 , 1);
    motor.setSpeed(1,1);
    motor.move();

    logger.log("Left current: %d, Right current: %d, Left target: %d, Right target: %d",
    (int) (motor.getCurrentLeftSpeed() * 100),
    (int) (motor.getCurrentRightSpeed() * 100),
    (int) (motor.getTargetLeftSpeed()  * 100),
    (int) (motor.getTargetRightSpeed() * 100));

    logger.log("Left speed: %d, Right speed: %d, Left distance: %d, Right distance: %d",
      (int) (motor.getTrueLeftSpeed() * 100),
      (int) (motor.getTrueRightSpeed() * 100),
      (int) (motor.getLeftDistance() * 100),
      (int) (motor.getRightDistance() * 100));
    
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

    logger.log("Left value: %d, Right value: %d, Left speed: %d, Right speed: %d",
      (int) (leftValue * 1000),
      (int) (rightValue * 1000),
      (int) (motor.getTrueLeftSpeed() * 1000),
      (int) (motor.getTrueRightSpeed() * 1000));


    logger.log("Left current: %d, Right current: %d, Left target: %d, Right target: %d",
    (int) (motor.getCurrentLeftSpeed() * 100),
    (int) (motor.getCurrentRightSpeed() * 100),
    (int) (motor.getTargetLeftSpeed()  * 100),
    (int) (motor.getTargetRightSpeed() * 100));
    motor.move();
  } else if (robotState ==  CoinState) {
    
  } else if (robotState ==  PushButtonState) {
    
  } else if (robotState ==  MoleState) {
    distance.updateDistance();
    float cm = distance.getDistance();
    logger.log("mm: %d", (int) (cm * 10));
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

    /*if(cm > 0) {
      motor.followWall(
        10.0,  // targetDistance
        cm,  // currentDistance
        .45,  // turning radius
        0.2, // average speed
        0.15    // correction factor
      );
    } else {
      motor.setTargetSpeed(0,0);
    }*/

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


