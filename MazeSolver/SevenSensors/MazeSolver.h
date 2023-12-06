#ifndef MAZESOLVER_H_INCLUDED
#define MAZESOLVER_H_INCLUDED
#include "LineFollower.h"
#include "Stack.h"

#define SHARP_LEFT_ERROR 101
#define SHARP_RIGHT_ERROR 102
#define CHOICE_OF_T_ERROR 103
#define STOP_ERROR 104
#define TURN_SPEED 120
#define VERIFICATION_DELAY 100

class MazeSolver : public LineFollower {
    Stack* path;
    byte 
        sharpLeftTurnSensorPin,
        sharpRightTurnSensorPin;
    public:
        bool isOn;
        MazeSolver(short initialSpeed, byte maxSpeed, byte numberOfSensors, int maxPathLength);
        float calculateError();
        void passLine();
        void turnToLeft();
        void turnToRight();
        void makeU();
        void choosePath();
        void solver();
        void setSharpTurnSensors(byte leftSensorPin, byte rightSensorPin);
};

MazeSolver::MazeSolver(short inicialSpeed, byte maxSpeed, byte numberOfSensors, int maxPathLength) : LineFollower(inicialSpeed, maxSpeed, numberOfSensors) {
    path = new Stack(maxPathLength);
}

float MazeSolver::calculateError() {
    readSensors();
    bool 
        sharpLeftTurnSensorValue = !digitalRead(sharpLeftTurnSensorPin),
        sharpRightTurnSensorValue = !digitalRead(sharpRightTurnSensorPin),
        centralSensorValue = sensorsInBlack[2];

    if (sharpLeftTurnSensorValue && centralSensorValue && sharpRightTurnSensorValue) 
        return CHOICE_OF_T_ERROR;
    if (sharpLeftTurnSensorValue && !centralSensorValue && sharpRightTurnSensorValue)
        return STOP_ERROR;
    if (sharpLeftTurnSensorValue)
        return SHARP_LEFT_ERROR;
    if (sharpRightTurnSensorValue)
        return SHARP_RIGHT_ERROR;
    return LineFollower::calculateError();
}

void MazeSolver::passLine() {
    setSpeed(TURN_SPEED);
    forward();
    delay(100);
}

void MazeSolver::turnToLeft() {
    passLine();
    rotateLeft();
    do readSensors();
    while (sensorsInBlack[2]);
    do readSensors();
    while (!sensorsInBlack[2]);
    stop();
    delay(1000);
    path->push('L');
}

void MazeSolver::turnToRight() {
    passLine();
    rotateRight();
    do readSensors();
    while (sensorsInBlack[2]);
    do readSensors();
    while (!sensorsInBlack[2]);
    stop();
    delay(1000);
    path->push('R');
}

void MazeSolver::makeU() {
    setSpeed(TURN_SPEED);
    rotateRight();
    do readSensors();
    while (!sensorsInBlack[2]);
    stop();
    delay(1000);
    path->push('U');
}

void MazeSolver::choosePath() {
    path->push('T');
    turnToLeft();
}

void MazeSolver::solver() {
    switch(round(calculateError())) {
        case MAKE_U_ERROR: // Make U (180°)
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else makeU();
            break;
        case SHARP_LEFT_ERROR: // Turn to left (90°)
            // if (path->get(-3) == 'T' && path->get(-2) == 'R' && path->get(-1) == 'U')
            //     break;
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else turnToLeft();
            break;
        case SHARP_RIGHT_ERROR: // Turn to right (90°)
            // if (path->get(-3) == 'T' && path->get(-2) == 'L' && path->get(-1) == 'U')
            //     break;
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else turnToRight();
            break;
        case CHOICE_OF_T_ERROR: // Choice of T
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else choosePath();
            break;
        case STOP_ERROR:
            stop();
            break;
        default:
            followLine();
    }
}

void MazeSolver::setSharpTurnSensors(byte leftSensorPin, byte rightSensorPin) {
    pinMode(leftSensorPin, INPUT);
    pinMode(rightSensorPin, INPUT);
    sharpLeftTurnSensorPin = leftSensorPin;
    sharpRightTurnSensorPin = rightSensorPin;
}
#endif