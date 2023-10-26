#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"
typedef unsigned char byte;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    byte speed, maxSpeed;
    public:
        Vehicle(byte spd, byte maxSpd);
        void setMotorsPins(byte firstLeftSensorPin, byte secondLeftSensorPin, byte firstRightSensorPin, byte secondRightSensorPin);
        void setSpeed(byte value);
        void setSpeed(byte leftMotorValue, byte rightMotorSpeed);
        void updateSpeed(byte value);
        void updateSpeed(byte leftMotorValue, byte rightMotorSpeed);
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
        byte getSpeed();
        byte getLeftMotorSpeed();
        byte getRightMotorSpeed();
};

Vehicle::Vehicle(byte spd, byte maxSpd) {
    speed = spd;
    maxSpeed = maxSpd;
}

void Vehicle::setMotorsPins(byte firstLeftSensorPin, byte secondLeftSensorPin, byte firstRightSensorPin, byte secondRightSensorPin) {
    leftMotor = new Motor(firstLeftSensorPin, secondLeftSensorPin, speed, maxSpeed);
    rightMotor = new Motor(firstRightSensorPin, secondRightSensorPin, speed, maxSpeed);
    pinMode(firstLeftSensorPin, OUTPUT);
    pinMode(secondLeftSensorPin, OUTPUT);
    pinMode(firstRightSensorPin, OUTPUT);
    pinMode(secondRightSensorPin, OUTPUT);
}

void Vehicle::setSpeed(byte value) {
    speed = value;
    setSpeed(value, value);
}

void Vehicle::setSpeed(byte leftMotorValue, byte rightMotorValue) {
    leftMotor->setSpeed(leftMotorValue);
    rightMotor->setSpeed(rightMotorValue);
}

void Vehicle::updateSpeed(byte value) {
    updateSpeed(value, value);
}

void Vehicle::updateSpeed(byte leftMotorValue, byte rightMotorValue) {
    setSpeed(getLeftMotorSpeed() + leftMotorValue, getRightMotorSpeed() + rightMotorValue);
}

byte Vehicle::getSpeed() {
    return (getLeftMotorSpeed() + getRightMotorSpeed())/2;
}

byte Vehicle::getLeftMotorSpeed() {
    return leftMotor->getSpeed();
}

byte Vehicle::getRightMotorSpeed() {
    return rightMotor->getSpeed();
}

void Vehicle::forward() {
    leftMotor->rotateClockwise();
    rightMotor->rotateCounterclockwise();
}

void Vehicle::backward() {
    leftMotor->rotateCounterclockwise();
    rightMotor->rotateClockwise();
}

void Vehicle::rotateLeft() {
    leftMotor->rotateCounterclockwise();
    rightMotor->rotateCounterclockwise();
}

void Vehicle::rotateRight() {
    leftMotor->rotateClockwise();
    rightMotor->rotateClockwise();
}

void Vehicle::stop() {
    leftMotor->stop();
    rightMotor->stop();
}

#endif
