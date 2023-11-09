#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"

typedef unsigned char byte;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    byte speed, maxSpeed;
    public:
        Vehicle(byte initialSpeed, byte maxSpeed);
        void setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin);
        void setSpeed(byte newSpeed);
        void setSpeed(byte leftMotorNewSpeed, byte rightMotorNewSpeed);
        void updateSpeed(byte deltaSpeed);
        void updateSpeed(byte leftMotorDeltaSpeed, byte rightMotorDeltaSpeed);
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
        byte getSpeed();
        byte getLeftMotorSpeed();
        byte getRightMotorSpeed();
};

Vehicle::Vehicle(byte initialSpeed, byte maxSpeed) {
    this->speed = initialSpeed;
    this->maxSpeed = maxSpeed;
}

void Vehicle::setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin) {
    leftMotor = new Motor(firstLeftMotorPin, secondLeftMotorPin, maxSpeed);
    rightMotor = new Motor(firstRightMotorPin, secondRightMotorPin, maxSpeed);
}

void Vehicle::setSpeed(byte newValue) {
    leftMotor->setSpeed(newValue);
    rightMotor->setSpeed(newValue);
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