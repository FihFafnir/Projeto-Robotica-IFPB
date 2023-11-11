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
        float currentError, innerSensorsError, outerSensorsError, centralSensorError;
        byte leftOuterSensorPin, leftInnerSensorPin, centralSensorPin, rightInnerSensorPin, rightOuterSensorPin;
        short initialSpeed;

        LineFollower(short initialSpeed, byte maxSpeed);
        float calculateError();
        float calculatePID();
        float getCurrentError();
        void followLine();
        void stop();
        void setErrorWeights(float newInnerSensorsError, float newOuterSensorsError, float centralSensorError);
        void setConstants(float kp, float ki, float kd);
        void setSensorsPins(byte newLeftOuterSensorPin, byte newLeftInnerSensorPin, byte newCentralSensorPin, byte newRightInnerSensorPin, byte newRightOuterSensorPin);
        byte readSensors();
};

LineFollower::LineFollower(short initialSpeed, byte maxSpeed) : Vehicle(initialSpeed, maxSpeed), initialSpeed(initialSpeed) {
    p = i = d = pid = currentError = previousError = 0;
}

float LineFollower::calculateError() {
    switch (readSensors()) {
        case 0b000001:
            return outerSensorsError;
        case 0b00011:
            return (outerSensorsError + innerSensorsError)/2;
        case 0b00010:
            return innerSensorsError;
        case 0b00110:
            return (innerSensorsError + centralSensorError)/2;
        case 0b00100:
            return centralSensorError;
        case 0b01100:
            return -(innerSensorsError + centralSensorError)/2;
        case 0b01000:
            return -innerSensorsError;
        case 0b11000:
            return -(outerSensorsError + innerSensorsError)/2;
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
    setSpeed(round(initialSpeed + pid), round(initialSpeed - pid));
    run();
}

void LineFollower::stop() {
    Vehicle::stop();
    setSpeed(initialSpeed);
    p = i = d = pid = currentError = previousError = 0;
}


void LineFollower::setErrorWeights(float newInnerSensorsError, float newOuterSensorsError, float newCentralSensorError) {
    innerSensorsError = newInnerSensorsError;
    outerSensorsError = newOuterSensorsError;
    centralSensorError = newCentralSensorError;
}

void LineFollower::setConstants(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void LineFollower::setSensorsPins(byte newLeftOuterSensorPin, byte newLeftInnerSensorPin, byte newCentralSensorPin, byte newRightInnerSensorPin, byte newRightOuterSensorPin) {
    leftOuterSensorPin = newLeftOuterSensorPin;
    leftInnerSensorPin = newLeftInnerSensorPin;
    centralSensorPin = newCentralSensorPin;
    rightInnerSensorPin = newRightInnerSensorPin;
    rightOuterSensorPin = newRightOuterSensorPin;
    pinMode(leftOuterSensorPin, INPUT);
    pinMode(leftInnerSensorPin, INPUT);
    pinMode(centralSensorPin, INPUT);
    pinMode(rightInnerSensorPin, INPUT);
    pinMode(rightOuterSensorPin, INPUT);
}

byte LineFollower::readSensors() {
    // Black: 1
    // White: 0
    bool 
        leftOuterSensorValue = !digitalRead(leftOuterSensorPin),
        leftInnerSensorValue = !digitalRead(leftInnerSensorPin),
        centralSensorValue = !digitalRead(centralSensorPin),
        rightInnerSensorValue = !digitalRead(rightInnerSensorPin),
        rightOuterSensorValue = !digitalRead(rightOuterSensorPin);
    return leftOuterSensorValue << 4 | leftInnerSensorValue << 3 | centralSensorValue << 2 | rightInnerSensorValue << 1 | rightOuterSensorValue;
}

#endif