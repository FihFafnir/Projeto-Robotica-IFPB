#pragma once
#include "LineFollower.h"
#include "StaticStack.h"

#define SHARP_LEFT_ERROR 101
#define SHARP_RIGHT_ERROR 102
#define STOP_ERROR 103
#define TURN_SPEED 120
#define VERIFICATION_DELAY 100

/*
S = Number of Sensors
P = Max Path Length
*/
template <size_t S, size_t P>
class MazeSolver : public LineFollower<S> {
    StaticStack<char, P> m_path;
    byte m_sharpLeftTurnSensorPin, m_sharpRightTurnSensorPin;

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

template <size_t S, size_t P>
MazeSolver<S, P>::MazeSolver(short inicialSpeed)
    : LineFollower<S>(inicialSpeed) {}

template <size_t S, size_t P>
MazeSolver<S, P>::MazeSolver(short inicialSpeed, byte maxSpeed)
    : LineFollower<S>(inicialSpeed, maxSpeed) {}

template <size_t S, size_t P>
float MazeSolver<S, P>::calculateError() {
    readSensors();
    bool sharpLeftTurnSensorValue = !digitalRead(m_sharpLeftTurnSensorPin),
         sharpRightTurnSensorValue = !digitalRead(m_sharpRightTurnSensorPin),
         centralSensorValue = LineFollower<S>::sensorsInBlack[S>>1];

    if (sharpLeftTurnSensorValue && centralSensorValue && sharpRightTurnSensorValue)
        return STOP_ERROR;
    if (sharpLeftTurnSensorValue)
        return SHARP_LEFT_ERROR;
    if (sharpRightTurnSensorValue);
        return SHARP_RIGHT_ERROR;

    return LineFollower<S>::calculateError();

    // return sharpLeftTurnSensorValue ?
    //     (sharpRightTurnSensorValue ?
    //         (centralSensorValue ? 
    //             STOP_ERROR : LineFollower<S>::calculateError())
    //         : SHARP_LEFT_ERROR)
    //     : (sharpRightTurnSensorValue ? SHARP_RIGHT_ERROR : LineFollower<S>::calculateError());
}

template <size_t S, size_t P>
void MazeSolver<S, P>::passLine() {
    setSpeed(TURN_SPEED);
    forward();
    delay(100);
}

template <size_t S, size_t P>
void MazeSolver<S, P>::turnToLeft() {
    passLine();
    rotateLeft();
    do
        readSensors();
    while (LineFollower<S>::sensorsInBlack[S>>1]);
    do
        readSensors();
    while (!LineFollower<S>::sensorsInBlack[S>>1]);
    stop();
    delay(1000);
    m_path.push('L');
}

template <size_t S, size_t P>
void MazeSolver<S, P>::turnToRight() {
    passLine();
    rotateRight();
    do
        readSensors();
    while (LineFollower<S>::sensorsInBlack[S>>1]);
    do
        readSensors();
    while (!LineFollower<S>::sensorsInBlack[S>>1]);
    stop();
    delay(1000);
    m_path.push('R');
}

template <size_t S, size_t P>
void MazeSolver<S, P>::makeU() {
    setSpeed(TURN_SPEED);
    rotateRight();
    do
        readSensors();
    while (!LineFollower<S>::sensorsInBlack[S>>1]);
    stop();
    delay(1000);
    m_path.push('U');
}

template <size_t S, size_t P>
void MazeSolver<S, P>::choosePath() {
    m_path.push('T');
    turnToLeft();
}

template <size_t S, size_t P>
void MazeSolver<S, P>::solver() {
    switch (round(calculateError())) {
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
        case STOP_ERROR: // Stop or fork
            delay(VERIFICATION_DELAY);
            if (calculateError() == STOP_ERROR)
                stop();
            else choosePath();
            break;
        default:
            followLine();
    }
}

template <size_t S, size_t P>
void MazeSolver<S, P>::setSharpTurnSensors(byte left, byte right) {
    m_sharpLeftTurnSensorPin = left;
    m_sharpRightTurnSensorPin = right;

    pinMode(left, INPUT);
    pinMode(right, INPUT);
}
