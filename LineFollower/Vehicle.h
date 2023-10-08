#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"
typedef unsigned char byte;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    byte speed, maxSpeed;
    public:
        Vehicle(byte spd, byte maxSpd);
        void setPins(byte pins[]);
        void setSpeed(byte value);
        void setSpeed(byte leftMotorValue, byte rightMotorSpeed);
        void updateSpeed(byte value);
        void updateSpeed(byte leftMotorValue, byte rightMotorSpeed);
        byte getSpeed();
        byte getLeftMotorSpeed();
        byte getRightMotorSpeed();
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
};

Vehicle::Vehicle(byte spd, byte maxSpd) {
    speed = spd;
    maxSpeed = maxSpd;
}

void Vehicle::setPins(byte pins[]) {
    leftMotor = new Motor(pins[0], pins[1], speed, maxSpeed);
    rightMotor = new Motor(pins[2], pins[3], speed, maxSpeed);
    pinMode(pins[0], OUTPUT);
    pinMode(pins[1], OUTPUT);
    pinMode(pins[2], OUTPUT);
    pinMode(pins[3], OUTPUT);
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
    leftMotor->rotateClockwise();
    rightMotor->rotateClockwise();
}

void Vehicle::rotateRight() {
    leftMotor->rotateCounterclockwise();
    rightMotor->rotateCounterclockwise();
}

void Vehicle::stop() {
    leftMotor->stop();
    rightMotor->stop();
}

#endif
