#ifndef MAZESOLVER_H_INCLUDED
#define MAZESOLVER_H_INCLUDED
#include "Vehicle.h"
#include "Stack.h"
typedef unsigned char byte;

class MazeSolver : public Vehicle {
    float 
        kp, ki, kd, 
        p, i, d, 
        pid, currentError, previousError, 
        middleSensorsError, outerSensorsError;
    byte initialSpeed, leftmostSensorPin, leftSensorPin, rightSensorPin, rightmostSensorPin;
    Stack* path;

    public:
        MazeSolver(byte initialSpeed, byte maxSpeed, int maxPathLength);
        float calculateError();
        float calculatePID();
        float getCurrentError();
        void followLine();
        void passLine();
        void turnToLeft();
        void turnToRight();
        void makeU();
        void choosePath();
        void stop();
        void setConstants(float kp, float ki, float kd);
        void setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError);
        void setSensorsPins(byte newLeftmostSensorPin, byte newLeftSensorPin, byte newRightSensorPin, byte newRightmostSensorPin);
};

MazeSolver::MazeSolver(byte initialSpeed, byte maxSpeed, int maxPathLength) : Vehicle(initialSpeed, maxSpeed), initialSpeed(initialSpeed) {
    path = new Stack(maxPathLength);
    p = i = d = pid = currentError = previousError = 0;
}

float MazeSolver::calculateError() {
    // In Line = 1
    bool 
        leftMostSensorValue = !digitalRead(leftmostSensorPin),
        leftSensorValue = !digitalRead(leftSensorPin),
        rightSensorValue = !digitalRead(rightSensorPin),
        rightMostSensorValue = !digitalRead(rightmostSensorPin);

    if (leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 100;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && rightMostSensorValue)
        return 101;
    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return 102;
    if (leftMostSensorValue && leftSensorValue && rightSensorValue && rightMostSensorValue)
        return 103;
    if (leftMostSensorValue && !leftSensorValue && !rightSensorValue && rightMostSensorValue)
        return 104;

    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && rightMostSensorValue)
        return outerSensorsError;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && rightMostSensorValue)
        return (outerSensorsError + middleSensorsError)/2;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return middleSensorsError;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 0;
    if (!leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -middleSensorsError;
    if (leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -(outerSensorsError + middleSensorsError)/2;
    if (leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -outerSensorsError;
}

float MazeSolver::calculatePID() {
    currentError = calculateError();
    p = currentError, 
    d = currentError - previousError;
    i += currentError;
    previousError = currentError;
    return kp*p + ki*i + kd*d;
}

float MazeSolver::getCurrentError() {
    currentError = calculateError();
    return currentError;
}

void MazeSolver::followLine() {
    pid = calculatePID();
    setSpeed(initialSpeed + pid, initialSpeed - pid);
    forward();
}

void MazeSolver::passLine() {
    setSpeed(initialSpeed);
    forward();
    
    do currentError = calculateError();
    while (currentError >= 100 && currentError <= 103);

    // delay(20*initialSpeed/3);
    delay(330);
}

void MazeSolver::turnToLeft() {
    passLine();
    // if (path->size() < 2 || path->get(-1) != 'B'|| path->get(-2) != 'R') {
        // if (currentError == 102 || (path->get(-1) == 'B' && path->get(-2) == 'S')) {
            do {
                rotateLeft();
                currentError = calculateError();
            } while (currentError < 100);
            do {
                rotateLeft();
                currentError = calculateError();
            } while (currentError != 0);
            path->push('L');
        // } else path->push('S');
    // }
}

void MazeSolver::turnToRight() {
    passLine();
    if (path->size() < 2 || path->get(-1) != 'B' || path->get(-2) != 'L') {
        if (currentError == 102 || (path->get(-1) == 'B' && path->get(-2) == 'S')) {
            do {
                rotateRight();
                currentError = calculateError();
            } while (currentError < 100);
            do {
                rotateRight();
                currentError = calculateError();
            } while (currentError != 0);
            path->push('R');
        } else path->push('S');
    }
}

void MazeSolver::choosePath() {
    path->push('T');
    turnToRight();
}

void MazeSolver::makeU() {
    path->push('U');
    setSpeed(initialSpeed);
    do {
        rotateRight();
        currentError = calculateError();
    } while (currentError != 0);
}

void MazeSolver::stop() {
    Vehicle::stop();
    setSpeed(initialSpeed);
    p = i = d = pid = currentError = previousError = 0;
}

void MazeSolver::setConstants(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void MazeSolver::setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError) {
    middleSensorsError = newMiddleSensorsError;
    outerSensorsError = newOuterSensorsError;
}

void MazeSolver::setSensorsPins(byte newLeftmostSensorPin, byte newLeftSensorPin, byte newRightSensorPin, byte newRightmostSensorPin) {
    leftmostSensorPin = newLeftmostSensorPin;
    leftSensorPin = newLeftSensorPin;
    rightSensorPin = newRightSensorPin;
    rightmostSensorPin = newRightmostSensorPin;
    pinMode(leftmostSensorPin, INPUT);
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
    pinMode(rightmostSensorPin, INPUT);
}

#endif