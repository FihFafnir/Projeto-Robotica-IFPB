#ifndef LINEFOLLOWER_H_INCLUDED
#define LINEFOLLOWER_H_INCLUDED
#include "Vehicle.h"
typedef unsigned char byte;

class LineFollower : public Vehicle {
    float 
        kp, ki, kd, 
        p, i, d, 
        pid, currentError, previousError, 
        sensorsError;
    byte initialSpeed, leftSensorPin, rightSensorPin;
    
    public:
        LineFollower(byte initialSpeed, byte maxSpeed);
        float calculateError();
        float calculatePID();
        void followLine();
        void stop();
        void setConstants(float newKp, float newKi, float newKd);
        void setErrorWeights(float newSensorsError);
        void setSensorsPins(byte newLeftSensorPin, byte newRightSensorPin);
};

LineFollower::LineFollower(byte initialSpeed, byte maxSpeed) : Vehicle(initialSpeed, maxSpeed), initialSpeed(initialSpeed) {
    stop();
}


float LineFollower::calculateError() {
    // In Line = 1
    bool 
        leftSensorValue = !digitalRead(leftSensorPin),
        rightSensorValue = !digitalRead(rightSensorPin);

    return sensorsError * (rightSensorValue - leftSensorValue);
}


float LineFollower::calculatePID() {
    currentError = calculateError();
    p = currentError, 
    d = currentError - previousError;
    i += currentError;
    previousError = currentError;
    return kp*p + ki*i + kd*d;
}

void LineFollower::followLine() {
    pid = calculatePID();
    setSpeed(initialSpeed + pid, initialSpeed - pid);
    forward();
}

void LineFollower::stop() {
    Vehicle::stop();
    setSpeed(initialSpeed);
    p = i = d = pid = currentError = previousError = 0;
}

void LineFollower::setConstants(float newKp, float newKi, float newKd) {
    kp = newKp;
    ki = newKi;
    kd = newKd;
}

void LineFollower::setErrorWeights(float newSensorsError) {
    sensorsError = newSensorsError;
}

void LineFollower::setSensorsPins(byte newLeftSensorPin, byte newRightSensorPin) {
    leftSensorPin = newLeftSensorPin;
    rightSensorPin = newRightSensorPin;
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
}


#endif