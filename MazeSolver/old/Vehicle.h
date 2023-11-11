#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"

typedef unsigned char byte;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    byte maxSpeed;
    float speed;
    public:
        Vehicle(byte initialSpeed, byte maxSpeed);
        void setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin);
        void setSpeed(float newSpeed);
        void setSpeed(float leftMotorNewSpeed, float rightMotorNewSpeed);
        void updateSpeed(float deltaSpeed);
        void updateSpeed(float leftMotorDeltaSpeed, float rightMotorDeltaSpeed);
        void run();
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
        float getSpeed();
        float getLeftMotorSpeed();
        float getRightMotorSpeed();
};

Vehicle::Vehicle(byte initialSpeed, byte maxSpeed) {
    this->speed = initialSpeed;
    this->maxSpeed = maxSpeed;
}

void Vehicle::setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin) {
    leftMotor = new Motor(firstLeftMotorPin, secondLeftMotorPin, maxSpeed);
    rightMotor = new Motor(firstRightMotorPin, secondRightMotorPin, maxSpeed);
}

void Vehicle::setSpeed(float newSpeed) {
    leftMotor->setSpeed(newSpeed);
    rightMotor->setSpeed(newSpeed);
}

void Vehicle::setSpeed(float newLeftMotorSpeed, float newRightMotorSpeed) {
    leftMotor->setSpeed(newLeftMotorSpeed);
    rightMotor->setSpeed(newRightMotorSpeed);
}

void Vehicle::updateSpeed(float deltaSpeed) {
    setSpeed(getSpeed() + deltaSpeed);
}

void Vehicle::updateSpeed(float leftMotorDeltaSpeed, float rightMotorDeltaSpeed) {
    setSpeed(getLeftMotorSpeed() + leftMotorDeltaSpeed, getRightMotorSpeed() + rightMotorDeltaSpeed);
}

float Vehicle::getSpeed() {
    return (getLeftMotorSpeed() + getRightMotorSpeed())/2;
}

float Vehicle::getLeftMotorSpeed() {
    return leftMotor->getSpeed();
}

float Vehicle::getRightMotorSpeed() {
    return rightMotor->getSpeed();
}

void Vehicle::run()
{
    if (getLeftMotorSpeed() > 0 && getRightMotorSpeed() > 0)
        forward();
    else if (getLeftMotorSpeed() < 0 && getRightMotorSpeed() < 0)
        backward();
    else if (getLeftMotorSpeed() > 0 && getRightMotorSpeed() < 0)
        rotateRight();
    else if (getLeftMotorSpeed() < 0 && getRightMotorSpeed() > 0)
        rotateLeft();
    else stop();
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