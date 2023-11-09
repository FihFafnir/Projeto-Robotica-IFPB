#include "MazeSolver.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 60

// Sensor Configuration
#define LEFTMOST_SENSOR_PIN 12
#define LEFT_SENSOR_PIN 11
#define RIGHT_SENSOR_PIN 4
#define RIGHTMOST_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PIN_1 9
#define LEFT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 20
#define KI 0
#define KD 60
// Error Weights
#define MIDDLE_SENSORS_ERROR 1
#define OUTER_SENSORS_ERROR 2.2

#define MAX_PATH_LENGTH 100

typedef unsigned char byte;

MazeSolver *ms = new MazeSolver(INITIAL_SPEED, MAX_SPEED, MAX_PATH_LENGTH);
bool end = true, solved = false;

void setup() {
    ms->setConstants(KP, KI, KD);
    ms->setErrorWeights(MIDDLE_SENSORS_ERROR, OUTER_SENSORS_ERROR);
    ms->setSensorsPins(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
    ms->setMotorsPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
}

void loop() {
    if (end) {
        ms->stop();
        if (ms->getCurrentError() == 0)
            end = false;
    } else switch(round(ms->getCurrentError())) {
        case 100: // Turn to left (90°)
            ms->turnToLeft();
            break;

        case 101: // Turn to right (90°)
            ms->turnToRight();
            break;

        case 102: // Make U (180°)
            ms->makeU();
            break;

        case 103: // Choice of T
            ms->choosePath();
            break;

        case 104: // Stop
            end = solved = true;
            break;
        default:
            ms->followLine();
    }
}
