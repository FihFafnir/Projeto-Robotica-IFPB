#ifndef LINEFOLLOWER_H_INCLUDED
#define LINEFOLLOWER_H_INCLUDED
#include "Vehicle.h"

typedef unsigned char byte;

class LineFollower : public Vehicle {
    float 
        kp, ki, kd, 
        p, i, d, 
        pid, currentError, previousError,
        middleSensorsError, outerSensorsError;
    byte initialSpeed, leftOuterSensorPin, leftInnerSensorPin, middleSensorPin, rightInnerSensorPin, rightOuterSensorPin;

    public:
        LineFollower(byte initialSpeed, byte maxSpeed);
        float calculateError();
        float calculatePID();
        float getCurrentError();
        void followLine();
        void stop();
        void setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError);
        void setConstants(float kp, float ki, float kd);
        void setSensorsPins(byte newLeftOuterSensorPin, byte newLeftInnerSensorPin, byte newMiddleSensorPin, byte newRightInnerSensorPin, byte newRightOuterSensorPin);

};

LineFollower::LineFollower(byte initialSpeed, byte maxSpeed) : Vehicle(initialSpeed, maxSpeed) {
    p = i = d = pid = currentError = previousError = 0;
}

float LineFollower::calculateError() {
    // In Line = 1
    bool 
        leftOuterSensorValue = !digitalRead(leftOuterSensorPin),
        leftInnerSensorValue = !digitalRead(leftInnerSensorPin),
        middleSensorValue = !digitalRead(middleSensorPin),
        rightInnerSensorValue = !digitalRead(rightInnerSensorPin),
        rightOuterSensorValue = !digitalRead(rightOuterSensorPin);

    if (!leftOuterSensorValue && !leftInnerSensorValue && !middleSensorValue && !rightInnerSensorValue && rightInnerSensorValue)
        return outerSensorsError;
    if (!leftOuterSensorValue && !leftInnerSensorValue && !middleSensorValue && rightInnerSensorValue && rightInnerSensorValue)
        return (outerSensorsError + middleSensorsError)/2;
    if (!leftOuterSensorValue && !leftInnerSensorValue && !middleSensorValue && rightInnerSensorValue && !rightInnerSensorValue)
        return middleSensorsError;
    if (!leftOuterSensorValue && !leftInnerSensorValue && middleSensorValue && !rightInnerSensorValue && !rightInnerSensorValue)
        return 0;
    if (!leftOuterSensorValue && leftInnerSensorValue && !middleSensorValue && !rightInnerSensorValue && !rightInnerSensorValue)
        return -middleSensorsError;
    if (leftOuterSensorValue && leftInnerSensorValue && !middleSensorValue && !rightInnerSensorValue && !rightInnerSensorValue)
        return -(outerSensorsError + middleSensorsError)/2;
    if (leftOuterSensorValue && !leftInnerSensorValue && !middleSensorValue && !rightInnerSensorValue && !rightInnerSensorValue)
        return -outerSensorsError;

}

float LineFollower::calculatePID() {
    currentError = calculateError();
    p = currentError, 
    d = currentError - previousError;
    i += currentError;
    previousError = currentError;
    return kp*p + ki*i + kd*d;
}

float LineFollower::getCurrentError() {
    currentError = calculateError();
    return currentError;
}

void LineFollower::followLine() {
    pid = calculatePID();
    setSpeed(initialSpeed - pid, initialSpeed + pid);
    forward();
}

void LineFollower::stop() {
    Vehicle::stop();
    setSpeed(initialSpeed);
    p = i = d = pid = currentError = previousError = 0;
}


void LineFollower::setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError) {
    middleSensorsError = newMiddleSensorsError;
    outerSensorsError = newOuterSensorsError;
}

void LineFollower::setConstants(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void LineFollower::setSensorsPins(byte newLeftOuterSensorPin, byte newLeftInnerSensorPin, byte newMiddleSensorPin, byte newRightInnerSensorPin, byte newRightOuterSensorPin) {
    leftOuterSensorPin = newLeftOuterSensorPin;
    leftInnerSensorPin = newLeftInnerSensorPin;
    middleSensorPin = newMiddleSensorPin;
    rightInnerSensorPin = newRightInnerSensorPin;
    rightOuterSensorPin = newRightOuterSensorPin;
    pinMode(leftOuterSensorPin, INPUT);
    pinMode(leftInnerSensorPin, INPUT);
    pinMode(middleSensorPin, INPUT);
    pinMode(rightInnerSensorPin, INPUT);
    pinMode(rightOuterSensorPin, INPUT);
}

#endif