#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 80

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
#define KP 30
#define KI 0.00003
#define KD 60

// Error Weights Configuration
#define LEFTMOST_SENSOR_ERROR 3
#define LEFT_SENSOR_ERROR 1
#define RIGHT_SENSOR_ERROR 1
#define RIGHTMOST_SENSOR_ERROR 3

typedef unsigned char byte;

Vehicle *myV = new Vehicle(INITIAL_SPEED, MAX_SPEED);
float p = 0, i = 0, d = 0, pid = 0, currentError = 0, previousError = 0;

int calculateError(int leftMostSensorPin, int leftSensorPin, int rightSensorPin, int rightMostSensorPin) {
    bool 
        leftMostSensorValue = digitalRead(leftMostSensorPin),
        leftSensorValue = digitalRead(leftSensorPin),
        rightSensorValue = digitalRead(rightSensorPin),
        rightMostSensorValue = digitalRead(rightMostSensorPin);

    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && rightMostSensorValue)
        return 2.2;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && rightMostSensorValue)
        return 1.65;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 1;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 0;
    if (!leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -1;
    if (leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -1.65;
    if (leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -2.2;
    if (leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 100;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && rightMostSensorValue)
        return 101;
    // if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
    //     return 102;
    return 103;
}

int calculatePID() {
    currentError = calculateError(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
    p = currentError, 
    d = currentError - previousError;
    i += currentError;
    previousError = currentError;
    return KP*p + KI*i + KD*d;
}

void setup() {
    byte pins[] = {LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2};
    myV->setPins(pins);
    pinMode(LEFTMOST_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(RIGHTMOST_SENSOR_PIN, INPUT);
}

void loop() {
    pid = calculatePID();
    if (currentError == 100) { // Turn to left (90°)
        myV->setSpeed(80);
        myV->forward();
        delay(200);
        do {
            myV->rotateLeft();
            currentError = calculateError(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
        } while(currentError != 0);
    } else if (currentError == 101) { // Turn to right (90°)
        myV->setSpeed(80);
        myV->forward();
        delay(200);
        do {
            myV->rotateRight();
            currentError = calculateError(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
        } while(currentError != 0);
    } else if (currentError == 102) { // Make U (180°)
        myV->setSpeed(80);
        do {
            myV->rotateRight();
            currentError = calculateError(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN);
        } while(!currentError);
    } else if (currentError == 103) { // Stop
        myV->stop();
        myV->setSpeed(INITIAL_SPEED);
        currentError = previousError = pid = 0;
    } else {
        myV->setSpeed(INITIAL_SPEED + pid, INITIAL_SPEED - pid);
        myV->forward();
    }
}
