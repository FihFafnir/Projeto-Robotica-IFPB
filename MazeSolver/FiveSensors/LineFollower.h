#ifndef LINEFOLLOWER_H_INCLUDED
#define LINEFOLLOWER_H_INCLUDED
#include "Vehicle.h"

typedef unsigned char byte;

class LineFollower : public Vehicle {
    float 
        kp, ki, kd, 
        p, i, d, 
        pid, previousError;

    public:
        float currentError, middleSensorsError, outerSensorsError, centralSensorError;
        byte initialSpeed, leftOuterSensorPin, leftInnerSensorPin, middleSensorPin, rightInnerSensorPin, rightOuterSensorPin;
        LineFollower(byte initialSpeed, byte maxSpeed);
        float calculateError();
        float calculatePID();
        float getCurrentError();
        void followLine();
        void stop();
        void setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError, float centralSensorError);
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

    switch (leftOuterSensorValue << 4 | leftInnerSensorValue << 3 | middleSensorValue << 2 | rightInnerSensorValue << 1 | rightOuterSensorValue) {
        case 0b000001:
            return outerSensorsError;
        case 0b00011:
            return (outerSensorsError + middleSensorsError)/2;
        case 0b00010:
            return middleSensorsError;
        case 0b00110:
            return (middleSensorsError + centralSensorError)/2;
        case 0b00100:
            return centralSensorError;
        case 0b01100:
            return -(middleSensorsError + centralSensorError)/2;
        case 0b01000:
            return -middleSensorsError;
        case 0b11000:
            return -(outerSensorsError + middleSensorsError)/2;
        case 0b10000:
            return -outerSensorsError;
    }
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
    setSpeed(initialSpeed + pid, initialSpeed - pid);
    run();
}

void LineFollower::stop() {
    Vehicle::stop();
    setSpeed(initialSpeed);
    p = i = d = pid = currentError = previousError = 0;
}


void LineFollower::setErrorWeights(float newMiddleSensorsError, float newOuterSensorsError, float newCentralSensorError) {
    middleSensorsError = newMiddleSensorsError;
    outerSensorsError = newOuterSensorsError;
    centralSensorError = newCentralSensorError;
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