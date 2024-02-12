#pragma once
#include "LineFollower.h"

#define SHARP_LEFT_ERROR 101
#define SHARP_RIGHT_ERROR 102
#define STOP_ERROR 103
#define TURN_SPEED 100
#define VERIFICATION_DELAY 100

// S = Number of Sensors
template <size_t S>
class MazeSolver : public LineFollower<S> {
    byte 
        m_sharpLeftTurnSensorPin,
        m_sharpRightTurnSensorPin;
    public:
        MazeSolver(short initialSpeed);
        MazeSolver(short initialSpeed, byte maxSpeed);

        float calculateError();
        void passLine();
        void turnToLeft();
        void turnToRight();
        void makeU();
        void choosePath();
        void solver();
        void setSharpTurnSensors(byte left, byte right);
};

template <size_t S>
MazeSolver<S>::MazeSolver(short inicialSpeed) : LineFollower<S>(inicialSpeed) {}

template <size_t S>
MazeSolver<S>::MazeSolver(short inicialSpeed, byte maxSpeed) : LineFollower<S>(inicialSpeed, maxSpeed) {}

template <size_t S>
float MazeSolver<S>::calculateError() {
    bool 
        sharpLeftTurnSensorValue = !digitalRead(m_sharpLeftTurnSensorPin),
        sharpRightTurnSensorValue = !digitalRead(m_sharpRightTurnSensorPin),
        centralSensorValue = (readSensors() >> (S >> 1)) & 1;

    if (sharpLeftTurnSensorValue && centralSensorValue && sharpRightTurnSensorValue) 
        return STOP_ERROR;
    if (sharpLeftTurnSensorValue)
        return SHARP_LEFT_ERROR;
    if (sharpRightTurnSensorValue)
        return SHARP_RIGHT_ERROR;
    return LineFollower<S>::calculateError();
}

template <size_t S>
void MazeSolver<S>::passLine() {
    setSpeed(TURN_SPEED);
    forward();
    delay(100);
}

template <size_t S>
void MazeSolver<S>::turnToLeft() {
    bool centralSensorValue;
    passLine();
    rotateLeft();
    do centralSensorValue = (readSensors() >> (S >> 1)) & 1;
    while (centralSensorValue);
    do centralSensorValue = (readSensors() >> (S >> 1)) & 1;
    while (!centralSensorValue);
    stop();
    delay(1000);
}

template <size_t S>
void MazeSolver<S>::turnToRight() {
    bool centralSensorValue;
    passLine();
    rotateRight();
    do centralSensorValue = (readSensors() >> (S >> 1)) & 1;
    while (centralSensorValue);
    do centralSensorValue = (readSensors() >> (S >> 1)) & 1;
    while (!centralSensorValue);
    stop();
    delay(1000);
}

template <size_t S>
void MazeSolver<S>::makeU() {
    bool centralSensorValue;
    setSpeed(TURN_SPEED);
    rotateRight();
    do centralSensorValue = (readSensors() >> (S >> 1)) & 1;
    while (!centralSensorValue);
    stop();
    delay(1000);
}

template <size_t S>
void MazeSolver<S>::choosePath() {
    turnToLeft();
}

template <size_t S>
void MazeSolver<S>::solver() {
    switch(round(calculateError())) {
        case MAKE_U_ERROR: // Make U (180°)
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else makeU();
            break;
        case SHARP_LEFT_ERROR: // Turn to left (90°)
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else turnToLeft();
            break;
        case SHARP_RIGHT_ERROR: // Turn to right (90°)
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else turnToRight();
            break;
        case STOP_ERROR: // Stop or Choice of T
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else choosePath();
            break;
        default:
            followLine();
    }
}

template <size_t S>
void MazeSolver<S>::setSharpTurnSensors(byte left, byte right) {
    pinMode(left, INPUT);
    pinMode(right, INPUT);
    m_sharpLeftTurnSensorPin = left;
    m_sharpRightTurnSensorPin = right;
}
