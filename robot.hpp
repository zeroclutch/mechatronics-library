#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef LINES_H
#include "lines.hpp"
#endif

#ifndef MOTOR_H
#include "motor.hpp"
#endif

#ifndef CALIBRATION_MODE
#define CALIBRATION_MODE 0
#endif

#ifndef ROBOT_MAX_MODULES
#define ROBOT_MAX_MODULES 10
#endif

#ifndef ROBOT_H
#define ROBOT_H

enum MODULES {
  ColorsModule,
  MotorModule,
  LinesModule,
  DistanceModule,
  TCSModule
};

extern enum MODULES Modules;

enum ROBOT_STATES {
  InitializeState,
  IdleState,
  SeekLineState,
  FollowLineState,
  SeekCoinState,
  AlignCoinState,
  CoinState,
  SeekCrossState,
  SeekButtonState,
  MoleColorState,
  SeekMoleState,
  CenterRobotState,
  EndState,

  // Other states
  CalibrateState
};

enum ARENA_ITEMS {
  MoleGreen,
  MoleBlue,
  MoleWhite,
  RedButton,
  MoleRed,
  MolePurple,
  MoleYellow
};

extern enum ROBOT_STATES RobotStates;
extern enum ARENA_ITEMS ArenaItems;

class Robot: public RobotModule {
  private:
    RobotModule *modules[ROBOT_MAX_MODULES];
    int moduleCount = 0;
    int state = 0;

    int position = RedButton;
    int targetPosition = MoleYellow;

    const uint8_t *LED_PINS;
    uint8_t LED_PIN_COUNT;

  public:
    Robot(const uint8_t *LED_PINS, uint8_t LED_PIN_COUNT);
    ~Robot();

    bool initialize();
    bool systemsCheck();
    void addModule(RobotModule *module, int index);

    void setState(int state);
    int getState();

    int getPosition();
    void setPosition(int position);

    int getTargetPosition();
    void setTargetPosition(int position);

    void nextPosition();
    void previousPosition();

    void setLEDs(int state);

    void followLine(float averageSpeed, float turnSpeed);

};

#endif