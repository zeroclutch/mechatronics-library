// Exports a robot structure with all sensor information and methods.
#include "robot.hpp"
#include <assert.h>

Robot::Robot() {
  moduleCount = 0;
  name = "Robot";
}

Robot::~Robot() {
  free(modules);
}

bool Robot::initialize() {
  logger->log("%d modules identified...", moduleCount);

  for(int i = 0; i < moduleCount; i++) {
    if(!modules[i]->initialize()) {
      logger->log("Initialization failed on module %d", i);
      return false;
    }

    logger->log("Successfully initialized module %d", i);
  }
}

bool Robot::systemsCheck() {
  logger->log("Running systems check on identified modules...");

  for(int i = 0; i < moduleCount; i++) {
    if(!modules[i]->systemsCheck()) {
      logger->log("System module check failed on module %i", i);
      return false;
    }

    logger->log("Successfully checked module %i", i);
  }
}

void Robot::addModule(RobotModule *module, int index) {
  logger->log("Adding module: %s", module->name);

  assert(moduleCount <= ROBOT_MAX_MODULES);
  assert(index <= ROBOT_MAX_MODULES);

  module->attachLogger(logger);

  modules[index] = module;
  moduleCount++;
}

void Robot::setState(int newState) {
  logger->log("Changing state from %d to %d", state, newState);
  state = newState;
}

int Robot::getState() {
  return state;
}