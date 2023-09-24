#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED
#include "Motor.h"
typedef unsigned short ushort;

class Vehicle {
    Motor *leftMotor, *rightMotor;
    ushort speed, maxSpeed;
    public:
        Vehicle(ushort spd, ushort maxSpd);
        void setPins(ushort pins[]);
        void setSpeed(ushort value);
        ushort getSpeed();
        void forward();
        void backward();
        void rotateLeft();
        void rotateRight();
        void stop();
};

Vehicle::Vehicle(ushort spd, ushort maxSpd) {
    speed = spd;
    maxSpeed = maxSpd;
}

void Vehicle::setPins(ushort pins[]) {
    leftMotor = new Motor(pins[0], pins[1], speed, maxSpeed);
    rightMotor = new Motor(pins[2], pins[3], speed, maxSpeed);
    pinMode(pins[0], OUTPUT);
    pinMode(pins[1], OUTPUT);
    pinMode(pins[2], OUTPUT);
    pinMode(pins[3], OUTPUT);
}

void Vehicle::setSpeed(ushort value) {
    speed = min(value, maxSpeed);
    leftMotor->setSpeed(speed);
    rightMotor->setSpeed(speed);
}

ushort Vehicle::getSpeed() {
    return speed;
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
