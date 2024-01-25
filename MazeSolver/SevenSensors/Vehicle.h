#pragma once
#include "Motor.h"

class Vehicle {
    Motor m_left, m_right;
    short m_speed;
    byte m_maxSpeed;

    public: 
        Vehicle();
        Vehicle(byte maxSpeed);
        void setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin);
        
        void setSpeed(short speed);
        void setSpeed(short leftMotorSpeed, short rightMotorSpeed);
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

Vehicle::Vehicle() : m_maxSpeed(MAX_SPEED_DEFAULT) {}

Vehicle::Vehicle(byte maxSpeed) : m_maxSpeed(maxSpeed) {}

void Vehicle::setMotorsPins(byte firstLeftMotorPin, byte secondLeftMotorPin, byte firstRightMotorPin, byte secondRightMotorPin) {
    m_left.setPins(firstLeftMotorPin, secondLeftMotorPin);
    m_right.setPins(firstRightMotorPin, secondRightMotorPin);
    setSpeed(m_speed);
}

void Vehicle::run() {
    m_left.rotate();
    m_right.rotate();
}

void Vehicle::forward() {
    m_left.rotate(Motor::REVERSE);
    m_right.rotate(Motor::NORMAL);
}

void Vehicle::backward() {
    m_left.rotate(Motor::NORMAL);
    m_right.rotate(Motor::REVERSE);
}

void Vehicle::rotateLeft() {
    m_left.rotate(Motor::NORMAL);
    m_right.rotate(Motor::NORMAL);
}

void Vehicle::rotateRight() {
    m_left.rotate(Motor::REVERSE);
    m_right.rotate(Motor::REVERSE);
}

void Vehicle::stop() {
    m_left.stop();
    m_right.stop();
}

void Vehicle::setSpeed(short speed) {
    m_left.setSpeed(-speed);
    m_right.setSpeed(speed);
}

void Vehicle::setSpeed(short leftMotorSpeed, short rightMotorSpeed) {
    m_left.setSpeed(-leftMotorSpeed);
    m_right.setSpeed(rightMotorSpeed);
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
    return m_left.getSpeed();
}

short Vehicle::getRightMotorSpeed() {
    return m_right.getSpeed();
}
