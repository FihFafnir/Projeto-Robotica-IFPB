#ifndef MAZESOLVER_H_INCLUDED
#define MAZESOLVER_H_INCLUDED
#include "LineFollower.h"
#include "Stack.h"

#define MAKE_U_ERROR 100
#define SHARP_LEFT_ERROR 101
#define SHARP_RIGHT_ERROR 102
#define CHOICE_OF_T_ERROR 103

typedef unsigned char byte;

class MazeSolver : public LineFollower {
    Stack* path;
    public:
        bool isOn;
        MazeSolver(byte initialSpeed, byte maxSpeed, int maxPathLength);
        float calculateError();
        float getCurrentError();
        void off();
        void on();
        void passLine();
        void turnToLeft();
        void turnToRight();
        void makeU();
        void choosePath();
        void solver();
};

MazeSolver::MazeSolver(byte inicialSpeed, byte maxSpeed, int maxPathLength) : LineFollower(inicialSpeed, maxSpeed) {
    path = new Stack(maxPathLength);
    off();
}

float MazeSolver::calculateError() {
    // In Line = 1
    bool 
        leftOuterSensorValue = !digitalRead(leftOuterSensorPin),
        leftInnerSensorValue = !digitalRead(leftInnerSensorPin),
        middleSensorValue = !digitalRead(middleSensorPin),
        rightInnerSensorValue = !digitalRead(rightInnerSensorPin),
        rightOuterSensorValue = !digitalRead(rightOuterSensorPin);

    switch (leftOuterSensorValue << 4 | leftInnerSensorValue << 3 | middleSensorValue << 2 | rightInnerSensorValue << 1 | rightOuterSensorValue) {
        case 0b000001:
            return outerSensorsError;
        case 0b00011:
            return (outerSensorsError + middleSensorsError)/2;
        case 0b00010:
            return middleSensorsError;
        case 0b00110:
            return (middleSensorsError + centralSensorError)/2;
        case 0b00100:
            return centralSensorError;
        case 0b01100:
            return -(middleSensorsError + centralSensorError)/2;
        case 0b01000:
            return -middleSensorsError;
        case 0b11000:
            return -(outerSensorsError + middleSensorsError)/2;
        case 0b10000:
            return -outerSensorsError;
        case 0b00000:
            return MAKE_U_ERROR;
        case 0b00111:
            return SHARP_RIGHT_ERROR;
        case 0b11100:
            return SHARP_LEFT_ERROR;
        case 0b11111:
            return CHOICE_OF_T_ERROR;
    }
}

float MazeSolver::getCurrentError() {
    currentError = calculateError();
    return currentError;
}

void MazeSolver::off() {
    isOn = false;
}

void MazeSolver::on() {
    isOn = true;
}

void MazeSolver::passLine() {
    setSpeed(initialSpeed);
    forward();
    
    do getCurrentError();
    while (currentError >= 100 && currentError <= 103);

    delay(100);
}

void MazeSolver::turnToLeft() {
    passLine();
    do rotateLeft();
    while (getCurrentError() < 100);
    do rotateLeft();
    while (getCurrentError() != 0);
    path->push('L');
}

void MazeSolver::turnToRight() {
    passLine();
    do rotateRight();
    while (getCurrentError() < 100);
    do rotateRight();
    while (getCurrentError() != 0);
    path->push('R');
}

void MazeSolver::makeU() {
    setSpeed(initialSpeed);
    do rotateRight();
    while (getCurrentError() != 0);
    path->push('U');
}

void MazeSolver::choosePath() {
    path->push('T');
    turnToRight();
}

void MazeSolver::solver() {
    switch(round(getCurrentError())) {
        case MAKE_U_ERROR: // Make U (180°)
            makeU();
            break;
        case SHARP_LEFT_ERROR: // Turn to left (90°)
            if (path->get(-3) == 'T' && path->get(-2) == 'R' && path->get(-1) == 'U')
                break;
            turnToLeft();
            break;
        case SHARP_RIGHT_ERROR: // Turn to right (90°)
                break;
            turnToRight();
            break;
        case CHOICE_OF_T_ERROR: // Choice of T
            choosePath();
            break;
        default:
            followLine();
    }
}
#endif