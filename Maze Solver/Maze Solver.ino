#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 150

// Sensor Configuration
#define LEFTMOST_SENSOR_PIN 13
#define LEFT_SENSOR_PIN 12
#define RIGHT_SENSOR_PIN 4
#define RIGHTMOST_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 10, 11)
#define LEFT_MOTOR_PIN_1 10
#define LEFT_MOTOR_PIN_2 11
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 15
#define KI 0
#define KD 25

// Error Weights Configuration
#define LEFTMOST_SENSOR_ERROR 3
#define LEFT_SENSOR_ERROR 1
#define RIGHT_SENSOR_ERROR 1
#define RIGHTMOST_SENSOR_ERROR 3

typedef unsigned char byte;

Vehicle *myV = new Vehicle(INITIAL_SPEED, MAX_SPEED);
int p = 0, i = 0, d = 0, pid = 0, currentError = 0, previousError = 0;

int calculateError(int leftMostSensorPin, int leftSensorPin, int rightSensorPin, int rightMostSensorPin) {
    bool 
        leftMostSensorValue = digitalRead(leftMostSensorPin),
        leftSensorValue = digitalRead(leftSensorPin),
        rightSensorValue = digitalRead(rightSensorPin),
        rightMostSensorValue = digitalRead(rightMostSensorPin);

    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return 102;

    if (leftSensorValue && rightSensorValue) {
        if (leftMostSensorValue && !rightMostSensorValue)
            return 100;
        if (!leftMostSensorValue && rightMostSensorValue)
            return 101;
    }

 
    return abs(leftMostSensorValue*LEFTMOST_SENSOR_ERROR - leftSensorValue*LEFT_SENSOR_ERROR) - abs(rightMostSensorValue*RIGHTMOST_SENSOR_ERROR - rightSensorValue*RIGHT_SENSOR_ERROR);
}

int calculatePID() {
    p = currentError, 
    d = currentError - previousError;
    i = currentError ? i + currentError : 0;
    return KP*p + KI*i + KD*d;
}

void setup() {
    byte pins[] = {RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PIN_1};
    myV->setPins(pins);
    pinMode(LEFTMOST_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(RIGHTMOST_SENSOR_PIN, INPUT);
}

void loop() {
    currentError = calculateError(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
    switch (currentError) {
    case 100: // Turn to left (90°)
        myV->setSpeed(150);
        myV->rotateLeft();
        delay(200);
        myV->stop();
        delay(1000);
        break;
    case 101: // Turn to right (90°)
        myV->setSpeed(150);
        myV->rotateRight();
        delay(200);
        myV->stop();
        delay(1000);
        break;
    case 102: // Make U (180°)
        myV->setSpeed(150);
        myV->rotateRight();
        delay(400);
        myV->stop();
        delay(1000);
        break;
    default:
        pid = calculatePID();
        previousError = currentError;
        myV->setSpeed(INITIAL_SPEED + pid, INITIAL_SPEED - pid);
        if (digitalRead(LEFTMOST_SENSOR_PIN) && digitalRead(LEFT_SENSOR_PIN) && digitalRead(RIGHT_SENSOR_PIN) && digitalRead(RIGHTMOST_SENSOR_PIN)) {
            myV->stop();
            myV->setSpeed(INITIAL_SPEED);
            currentError = previousError = pid = 0;
        } else myV->forward();
    }
}
