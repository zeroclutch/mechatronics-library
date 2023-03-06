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
  logger->log("[robot] %d modules identified...", moduleCount);

  for(int i = 0; i < moduleCount; i++) {
    if(modules[i] == NULL) continue;

    if(!modules[i]->initialize()) {
      logger->log("[robot] Initialization failed on module %s", modules[i]->name);
      return false;
    }

    logger->log("[robot] Successfully initialized module %s", modules[i]->name);
  }
  return true;
}

bool Robot::systemsCheck() {
  logger->log("[robot] Running systems check on identified modules...");

  for(int i = 0; i < moduleCount; i++) {
    if(modules[i] == NULL) continue;

    if(!modules[i]->systemsCheck()) {
      logger->log("[robot] System module check failed on module %s", modules[i]->name);
      return false;
    }

    logger->log("[robot] Successfully checked module %s", modules[i]->name);
  }
  return true;
}

void Robot::addModule(RobotModule *module, int index) {
  logger->log("[robot] Adding module: %s", module->name);

  if(index >= ROBOT_MAX_MODULES) {
    logger->log("[robot] Module index out of bounds: %d", index);
    return;
  }

  module->attachLogger(logger);

  modules[index] = module;
  moduleCount++;
}

void Robot::setState(int newState) {
  logger->log("[robot] Changing state from %d to %d", state, newState);
  state = newState;
}

int Robot::getState() {
  return state;
}