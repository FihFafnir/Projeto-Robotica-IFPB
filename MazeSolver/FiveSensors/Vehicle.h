#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"

typedef unsigned char byte;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    short speed;
    byte maxSpeed;
    public: 
        Vehicle(short initialSpeed, byte maxSpeed);
        void setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin);
        void setSpeed(short newSpeed);
        void setSpeed(short newLeftMotorSpeed, short newRightMotorSpeed);
        void updateSpeed(short speedDelta);
        void updateSpeed(short leftMotorSpeedDelta, short rightMotorSpeedDelta);
        void run();
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
        short getSpeed();
        short getLeftMotorSpeed();
        short getRightMotorSpeed();
};

Vehicle::Vehicle(short initialSpeed, byte maxSpeed) {
    this->speed = initialSpeed;
    this->maxSpeed = maxSpeed;
}

void Vehicle::setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin) {
    leftMotor = new Motor(firstLeftMotorPin, secondLeftMotorPin, maxSpeed);
    rightMotor = new Motor(firstRightMotorPin, secondRightMotorPin, maxSpeed);
    setSpeed(speed);
}

void Vehicle::run() {
    if (getLeftMotorSpeed() > 0 && getRightMotorSpeed() > 0)
        forward();
    else if (getLeftMotorSpeed() < 0 && getRightMotorSpeed() < 0)
        backward();
    else if (getLeftMotorSpeed() < 0 && getRightMotorSpeed() > 0)
        rotateLeft();
    else if (getLeftMotorSpeed() > 0 && getRightMotorSpeed() < 0)
        rotateRight();
}

void Vehicle::forward() {
    leftMotor->rotate(Motor::REVERSE);
    rightMotor->rotate(Motor::NORMAL);
}

void Vehicle::backward() {
    leftMotor->rotate(Motor::NORMAL);
    rightMotor->rotate(Motor::REVERSE);
}

void Vehicle::rotateLeft() {
    leftMotor->rotate(Motor::NORMAL);
    rightMotor->rotate(Motor::NORMAL);
}

void Vehicle::rotateRight() {
    leftMotor->rotate(Motor::REVERSE);
    rightMotor->rotate(Motor::REVERSE);
}

void Vehicle::stop() {
    leftMotor->stop();
    rightMotor->stop();
}

void Vehicle::setSpeed(short newSpeed) {
    leftMotor->setSpeed(newSpeed);
    rightMotor->setSpeed(newSpeed);
}

void Vehicle::setSpeed(short newLeftMotorSpeed, short newRightMotorSpeed) {
    leftMotor->setSpeed(newLeftMotorSpeed);
    rightMotor->setSpeed(newRightMotorSpeed);
}

void Vehicle::updateSpeed(short speedDelta) {
    setSpeed(getSpeed() + speedDelta);
}

void Vehicle::updateSpeed(short leftMotorSpeedDelta, short rightMotorSpeedDelta) {
    setSpeed(getLeftMotorSpeed() + leftMotorSpeedDelta, getRightMotorSpeed() + rightMotorSpeedDelta);
}

short Vehicle::getSpeed() {
    return (getLeftMotorSpeed() + getRightMotorSpeed())/2;
}

short Vehicle::getLeftMotorSpeed() {
    return leftMotor->getSpeed();
}

short Vehicle::getRightMotorSpeed() {
    return rightMotor->getSpeed();
}

#endif