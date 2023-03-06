#ifndef LOGGER_H
#include "logger.hpp"
#endif

#ifndef MODULE_H
#define MODULE_H

// Abstract class that represents a module
// A module is a piece of code that can be loaded to the robot
// It can be a sensor, a motor, a controller, etc.
class RobotModule {
public:

    char* name = "";
    Logger *logger;

    void attachLogger(Logger *logger) {
        this->logger = logger;
        logger->log("[module] Attached logger to module: %s", name);
    }

    // Initialize the module. This will be called in the setup() function
    // Return true if the initialization was successful
    virtual bool initialize() = 0;

    // Perform a shakedown of the module
    // Return true if the systemsCheck was successful
    virtual bool systemsCheck() = 0;
};

#endif