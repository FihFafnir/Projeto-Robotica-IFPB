#ifndef MAZESOLVER_H_INCLUDED
#define MAZESOLVER_H_INCLUDED
#include "LineFollower.h"
#include "Stack.h"

#define MAKE_U_ERROR 100
#define SHARP_LEFT_ERROR 101
#define SHARP_RIGHT_ERROR 102
#define CHOICE_OF_T_ERROR 103
#define TURN_SPEED 200

class MazeSolver : public LineFollower {
    Stack* path;
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
};

MazeSolver::MazeSolver(short inicialSpeed, byte maxSpeed, byte numberOfSensors, int maxPathLength) : LineFollower(inicialSpeed, maxSpeed, numberOfSensors) {
    path = new Stack(maxPathLength);
}

float MazeSolver::calculateError() {
    switch (readSensors()) {
        case 0b0001:
            return outerSensorsError;
        case 0b0011:
            return (outerSensorsError + innerSensorsError)/2;
        case 0b0010:
            return innerSensorsError;
        case 0b0110:
            return centralSensorError;
        case 0b0100:
            return -innerSensorsError;
        case 0b1100:
            return -(outerSensorsError + innerSensorsError)/2;
        case 0b1000:
            return -outerSensorsError;
        case 0b0000:
            return MAKE_U_ERROR;
        case 0b0111:
            return SHARP_RIGHT_ERROR;
        case 0b1110:
            return SHARP_LEFT_ERROR;
        case 0b1111:
            return CHOICE_OF_T_ERROR;
    }
}

void MazeSolver::passLine() {
    setSpeed(TURN_SPEED);
    forward();
    delay(250);
    stop();
    delay(300);
    setSpeed(TURN_SPEED);
}

void MazeSolver::turnToLeft() {
    passLine();
    rotateLeft();
    do readSensors();
    while (sensorsInBlack[1] || sensorsInBlack[2]);
    do readSensors();
    while (!sensorsInBlack[1] && !sensorsInBlack[2]);
    path->push('L');
}

void MazeSolver::turnToRight() {
    passLine();
    rotateRight();
    do readSensors();
    while (sensorsInBlack[1] || sensorsInBlack[2]);
    do readSensors();
    while (!sensorsInBlack[1] && !sensorsInBlack[2]);
    path->push('R');
}

void MazeSolver::makeU() {
    passLine();
    rotateRight();
    do readSensors();
    while (!sensorsInBlack[1] && !sensorsInBlack[2]);
    stop();
    delay(10000);
    setSpeed(getInicialSpeed());
    path->push('U');
}

void MazeSolver::choosePath() {
    path->push('T');
    stop();
}

void MazeSolver::solver() {
    switch(round(calculateError())) {
        case MAKE_U_ERROR: // Make U (180°)
            makeU();
            break;
        case SHARP_LEFT_ERROR: // Turn to left (90°)
            // if (path->get(-3) == 'T' && path->get(-2) == 'R' && path->get(-1) == 'U')
            //     break;
            // stop();
            // delay(300);
            // turnToLeft();
            // stop();
            // delay(300);
            break;
        case SHARP_RIGHT_ERROR: // Turn to right (90°)
            // if (path->get(-3) == 'T' && path->get(-2) == 'L' && path->get(-1) == 'U')
            //     break;
            // stop();
            // delay(300);
            // turnToRight();
            // stop();
            // delay(300);
            break;
        case CHOICE_OF_T_ERROR: // Choice of T
            // choosePath();
            break;
        default:
            followLine();
    }
}
#endif