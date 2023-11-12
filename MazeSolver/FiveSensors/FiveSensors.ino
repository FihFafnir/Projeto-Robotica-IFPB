#include "MazeSolver.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 70

// Sensor Configuration
#define LEFT_OUTER_SENSOR_PIN 12
#define LEFT_INNER_SENSOR_PIN 11
#define CENTRAL_SENSOR_PIN 8
#define RIGHT_INNER_SENSOR_PIN 4
#define RIGHT_OUTER_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PIN_1 10
#define LEFT_MOTOR_PIN_2 9
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 45 * ((INITIAL_SPEED+50)/100)
#define KI 0
#define KD 30

// Error Weights
#define CENTRAL_SENSOR_ERROR 0
#define INNER_SENSORS_ERROR 2
#define OUTER_SENSORS_ERROR 4

#define MAX_PATH_LENGTH 100

MazeSolver *ms = new MazeSolver(INITIAL_SPEED, MAX_SPEED, MAX_PATH_LENGTH);

void setup() {
    ms->setConstants(KP, KI, KD);
    ms->setErrorWeights(INNER_SENSORS_ERROR, OUTER_SENSORS_ERROR, CENTRAL_SENSOR_ERROR);
    ms->setSensorsPins(LEFT_OUTER_SENSOR_PIN, LEFT_INNER_SENSOR_PIN, CENTRAL_SENSOR_PIN, RIGHT_INNER_SENSOR_PIN, RIGHT_OUTER_SENSOR_PIN);
    ms->setMotorsPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
}

void loop() {
    if (
        !digitalRead(LEFT_OUTER_SENSOR_PIN) &&
        digitalRead(CENTRAL_SENSOR_PIN) &&
        !digitalRead(RIGHT_OUTER_SENSOR_PIN)
    ) ms->stop();
    else ms->followLine();
}