#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef DEBUG_ROBOT
#define DEBUG_ROBOT 0
#endif

#ifndef SWITCH_COUNT
#define SWITCH_COUNT 4
#endif

#ifndef MOTOR_H
#define MOTOR_H

#define PIN_COUNT 6

#include <assert.h>
#include <stdint.h>
#include "timeouts.h"

enum STATES {
  FORWARD     = 0,
  REVERSE     = 1,
  BRAKE       = 2,
  COAST       = 3,
  TURN_LEFT   = 4,
  TURN_RIGHT  = 5,
  PIVOT_LEFT  = 6,
  PIVOT_RIGHT = 7,

  LENGTH = 8
};

class Motor: public RobotModule {
  private:
    // IO pins
    uint8_t pinENA;
    uint8_t pinIN1;
    uint8_t pinIN2;
    uint8_t pinENB;
    uint8_t pinIN3;
    uint8_t pinIN4;
    uint8_t channelA;
    uint8_t channelB;
    uint8_t switchCount;
    uint8_t *switchPins;

    // State and speed machine
    int previousState;
    int currentState;

    float currentSpeed;
    float previousSpeed;

    // Internal variables
    int counter = 0;
    int lastChannelATime = 0;
    int lastChannelBTime = 0;
    int currChannelATime = 0;
    int currChannelBTime = 0;
    int countsPerRotation = 20;
    double circumference = 0.0635;
    double totalDistance = 0;
    bool started = false;

    void setPins(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t enB, uint8_t in3, uint8_t in4, int state);
    void setAllPins();

    int getPWMValue(uint8_t isEnabled);

    void printDistance();
    void printPins();
    void startCounting();

    void forward()    { currentState = FORWARD;     }
    void reverse()    { currentState = REVERSE;     }
    void brake()      { currentState = BRAKE;       }
    void coast()      { currentState = COAST;       }
    void turnLeft()   { currentState = TURN_LEFT;   }
    void turnRight()  { currentState = TURN_RIGHT;  }
    void pivotLeft()  { currentState = PIVOT_LEFT;  }
    void pivotRight() { currentState = PIVOT_RIGHT; }

  public:
    Motor(
        uint8_t pin_ENA,
        uint8_t pin_IN1,
        uint8_t pin_IN2,
        uint8_t pin_ENB,
        uint8_t pin_IN3,
        uint8_t pin_IN4,
        uint8_t CHANNEL_A,
        uint8_t CHANNEL_B,
        uint8_t switch_count,
        uint8_t *switchPins
    );
    bool initialize();
    bool systemsCheck();

    // Actions
    void move();

    // Setters
    void setSpeed(double speed);

    // Getters
    double getDistance();
    double getSpeed();
};

#endif