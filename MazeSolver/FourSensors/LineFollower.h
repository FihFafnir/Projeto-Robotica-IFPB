#ifndef LINEFOLLOWER_H_INCLUDED
#define LINEFOLLOWER_H_INCLUDED
#include "Vehicle.h"

#define CALIBRATION_TIME 10
#define MAKE_U_ERROR 100
class LineFollower : public Vehicle {
    float 
        kp, ki, kd, 
        p, i, d, 
        pid, previousError;
    short initialSpeed;

    public:
        float currentError, innerSensorsError, outerSensorsError, centralSensorError;
        byte leftOuterSensorPin, leftInnerSensorPin, centralSensorPin, rightInnerSensorPin, rightOuterSensorPin, *sensorsPins, numberOfSensors;
        int *sensorsMaxValues, *sensorsMinValues, *sensorsValues;
        bool *sensorsInBlack;

        LineFollower(short initialSpeed, byte maxSpeed, byte numberOfSensors);
        float calculateError();
        float calculatePID();
        void followLine();
        void stop();
        void setErrorWeights(float newInnerSensorsError, float newOuterSensorsError, float centralSensorError);
        void setConstants(float kp, float ki, float kd);
        void setSensorsPins(byte sensorsPins[]);
        void setInicialSpeed(short newInicialSpeed);
        void updateInicialSpeed(short inicialSpeedDelta);
        void calibrateSensors();
        short getInicialSpeed();
        byte readSensors();
};

LineFollower::LineFollower(short initialSpeed, byte maxSpeed, byte numberOfSensors) : Vehicle(initialSpeed, maxSpeed), initialSpeed(initialSpeed), numberOfSensors(numberOfSensors) {
    p = i = d = pid = currentError = previousError = 0;
    sensorsPins = new byte[numberOfSensors];
    sensorsMinValues = new int[numberOfSensors];
    sensorsMaxValues = new int[numberOfSensors];
    sensorsValues = new int[numberOfSensors];
    memset(sensorsPins, 0, sizeof(sensorsPins));
}

float LineFollower::calculateError() {
    switch (readSensors()) {
        case 0b01:
            return innerSensorsError;
        case 0b11:
            return centralSensorError;
        case 0b10:
            return -innerSensorsError;
        case 0b00:
            return MAKE_U_ERROR;
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

void LineFollower::setSensorsPins(byte sensorsPins[]) {
    for (byte i = 0; i < numberOfSensors; i++) {
        this->sensorsPins[i] = sensorsPins[i];
        pinMode(sensorsPins[i], INPUT);
    }
}

void LineFollower::setInicialSpeed(short newInicialSpeed) {
    initialSpeed = newInicialSpeed;
}

void LineFollower::updateInicialSpeed(short inicialSpeedDelta) {
    setInicialSpeed(getInicialSpeed() + inicialSpeedDelta);
}

void LineFollower::calibrateSensors() {
    unsigned long startTime = millis();
    for (byte i = 0; i < numberOfSensors; i++)
        sensorsValues[i] = sensorsMinValues[i] = sensorsMaxValues[i] = analogRead(sensorsPins[i]);
    setSpeed(100);
    rotateRight();
    while (millis() - startTime < CALIBRATION_TIME * 1000) {
        for (byte i = 0; i < numberOfSensors; i++) {
            sensorsValues[i] = analogRead(sensorsPins[i]);
            if (sensorsValues[i] < sensorsMinValues[i])
                sensorsMinValues[i] = sensorsValues[i];
            if (sensorsValues[i] > sensorsMaxValues[i])
                sensorsMaxValues[i] = sensorsValues[i];
        }
    }
    stop();
}

short LineFollower::getInicialSpeed() {
    return initialSpeed;
}

byte LineFollower::readSensors() {
    // Black: True
    // White: False
    byte result = 0, i;
    for (i = 0; i < numberOfSensors; i++) {
        sensorsValues[i] = digitalRead(sensorsPins[i]);
        sensorsInBlack[i] = sensorsValues[i];
        // sensorsInBlack[i] = !sensorsValues[i];
        //     sensorsValues[i] == sensorsMaxValues[i] ? false :
        //     sensorsValues[i] == sensorsMinValues[i] ? true : 
        //     sensorsValues[i] < (sensorsMaxValues[i] + sensorsMinValues[i])/2;
        result |= sensorsInBlack[i] << numberOfSensors - i - 1;
    }
    return result;
}

#endif