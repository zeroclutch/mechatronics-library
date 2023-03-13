#ifndef Arduino_h
#include "Arduino.h"
#endif

#ifndef MODULE_H
#include "module.hpp"
#endif

#ifndef TimerOne_h_
#include "TimerOne.h"
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

typedef struct MOTOR_SPEED {
  float left;
  float right;
} MotorSpeed;

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

enum WHEELS {
  WHEEL_LEFT,
  WHEEL_RIGHT
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
    uint8_t channelC;
    uint8_t channelD;
    uint8_t switchCount;
    uint8_t *switchPins;

    // State and speed machine
    int previousState;
    int currentState;

    // Internal variables
    float speedChangeRate = 0.01;  // 1% per tick
    const int wheelbase = 218; // 218mm
    const float wheelbaseMeters = 0.218; // 218mm
    const float wheelbaseCm = 21.8; // 218mm

    bool started = false;
    bool isAtMole = false;

    unsigned long lastSpeedUpdate = 0;
    const unsigned int SPEED_UPDATE_INTERVAL = 1000; // 1ms

    void setPins(uint8_t enA, uint8_t in1, uint8_t in2, uint8_t enB, uint8_t in3, uint8_t in4, int state);
    void setAllPins();

    bool isZero(float speed);
    bool isZero(float speed, float precision);
    float clampSpeed(float speed);

    void updatePreviousSpeed();
    void handleSpeedChange();
    int getPWMValue(uint8_t isEnabled, uint8_t wheel);

    int speedToInt(float speed);
    void printDistance();
    void printPins();
    void startCounting();

    void forward()    { currentState = FORWARD;     }
    void reverse()    { currentState = REVERSE;     }
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
        uint8_t CHANNEL_C,
        uint8_t CHANNEL_D,
        uint8_t switch_count,
        uint8_t *switchPins
    );
    ~Motor();

    bool initialize();
    bool systemsCheck();

    // Actions
    // This should be called in loop() continuously. It only updates the state.
    void move();

    // Reset the distance counters
    void resetCounters();

    // Sets the target wheel speed. Returns true if the speed was changed.
    bool setTargetSpeed(float left, float right);
    bool setTargetSpeed(MotorSpeed* speed);

    // Sets true wheel speed
    void setSpeed(float left, float right);
    void setSpeed(MotorSpeed* speed);

    float getCurrentLeftSpeed();
    float getCurrentRightSpeed();
    
    float getTargetLeftSpeed();
    float getTargetRightSpeed();
    
    // Sets the wheels to brake until the next setSpeed() call
    void brake() { currentState = BRAKE; }

    MotorSpeed* calculateSpeeds(MotorSpeed* dest, float averageSpeed, float angle);

    // Getters
    float getLeftDistance();
    float getRightDistance();

    float getTrueLeftSpeed();
    float getTrueRightSpeed();

    void followWall(float targetDistance, float currentDistance, float radius, float averageSpeed, int position);

    bool isForward() { return !(currentState == REVERSE); }

};

#endif