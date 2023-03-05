#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
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
  MotorModule
};

extern enum MODULES Modules;

enum ROBOT_STATES {
  InitializeState,
  SystemsCheckState,
  IdleState,
  CalibratingState,
  SeekLineState,
  FollowLineState,
  CoinState,
  MoleState,
  EndState
};

extern enum ROBOT_STATES RobotStates;

class Robot: public RobotModule {
  private:
    RobotModule *modules[ROBOT_MAX_MODULES];
    int moduleCount = 0;
    int state;

  public:
    Robot();
    ~Robot();

    bool initialize();
    bool systemsCheck();
    void addModule(RobotModule *module, int index);

    void setState(int state);
};

#endif