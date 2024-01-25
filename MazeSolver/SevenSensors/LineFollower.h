#pragma once
#include "Vehicle.h"

#define CALIBRATION_TIME 10
#define MAKE_U_ERROR 100

// S = Number of Sensors
template <size_t S>
class LineFollower : public Vehicle {
    float 
        m_kp, m_ki, m_kd, 
        m_p, m_i, m_d, 
        m_pid, m_previousError;
    short m_initialSpeed;

    public:
        float errors[1<<S];
        float currentError, innerSensorsError, outerSensorsError, centralSensorError;
        int sensorsMaxValues[S], sensorsMinValues[S], sensorsValues[S];
        byte sensorsPins[S];
        bool sensorsInBlack[S];

        LineFollower(short initialSpeed);
        LineFollower(short initialSpeed, byte maxSpeed);

        void setErrorWeights(float innerSensorsError, float outerSensorsError, float centralSensorError);
        void setConstants(float kp, float ki, float kd);
        void setSensorsPins(byte (&pins)[S]);
        void setInicialSpeed(short inicialSpeed);
        
        float calculateError();
        float calculatePID();
        void followLine();
        void stop();
        void updateInicialSpeed(short inicialSpeedDelta);
        void calibrateSensors();
        short getInicialSpeed();
        byte readSensors();
};

template <size_t S>
LineFollower<S>::LineFollower(short initialSpeed) : Vehicle(), m_initialSpeed(initialSpeed) {
    m_p = m_i = m_d = m_pid = currentError = m_previousError = 0;
}

template <size_t S>
LineFollower<S>::LineFollower(short initialSpeed, byte maxSpeed) : Vehicle(maxSpeed), m_initialSpeed(initialSpeed) {
    m_p = m_i = m_d = m_pid = currentError = m_previousError = 0;
}

template <size_t S>
void LineFollower<S>::setErrorWeights(float innerSensorsError, float outerSensorsError, float centralSensorError) {
    errors[0b00000] = MAKE_U_ERROR;
    errors[0b00001] = outerSensorsError;
    errors[0b00011] = (outerSensorsError + innerSensorsError) / 2;
    errors[0b00010] = innerSensorsError;
    errors[0b00110] = (innerSensorsError + centralSensorError) / 2;
    errors[0b00100] = centralSensorError;
    errors[0b01100] = -errors[0b00110];
    errors[0b01000] = -errors[0b00010];
    errors[0b11000] = -errors[0b00011];
    errors[0b10000] = -errors[0b00001];
}


template <size_t S>
void LineFollower<S>::setConstants(float kp, float ki, float kd) {
    m_kp = kp;
    m_ki = ki;
    m_kd = kd;
}

template <size_t S>
void LineFollower<S>::setSensorsPins(byte (&pins)[S]) {
    for (byte i = 0; i < S; i++) {
        sensorsPins[i] = pins[i];
        pinMode(pins[i], INPUT);
    }
}

template <size_t S>
void LineFollower<S>::setInicialSpeed(short inicialSpeed) {
    m_initialSpeed = inicialSpeed;
}


template <size_t S>
float LineFollower<S>::calculateError() {
    return errors[readSensors()];
}

template <size_t S>
float LineFollower<S>::calculatePID() {
    currentError = calculateError();
    m_p = currentError, 
    m_d = currentError - m_previousError;
    m_i += currentError;
    m_previousError = currentError;
    return m_kp*m_p + m_ki*m_i + m_kd*m_d;
}

template <size_t S>
void LineFollower<S>::followLine() {
    m_pid = calculatePID();
    setSpeed(round(m_initialSpeed + m_pid), round(m_initialSpeed - m_pid));
    run();
}

template <size_t S>
void LineFollower<S>::stop() {
    Vehicle::stop();
    setSpeed(m_initialSpeed);
    calculatePID();
}

template <size_t S>
void LineFollower<S>::updateInicialSpeed(short inicialSpeedDelta) {
    setInicialSpeed(m_initialSpeed + inicialSpeedDelta);
}

template <size_t S>
void LineFollower<S>::calibrateSensors() {
    unsigned long startTime = millis();
    for (byte i = 0; i < S; i++)
        sensorsValues[i] = sensorsMinValues[i] = sensorsMaxValues[i] = analogRead(sensorsPins[i]);
    setSpeed(100);
    rotateRight();
    while (millis() - startTime < CALIBRATION_TIME * 1000) {
        for (byte i = 0; i < S; i++) {
            sensorsValues[i] = analogRead(sensorsPins[i]);
            if (sensorsValues[i] < sensorsMinValues[i])
                sensorsMinValues[i] = sensorsValues[i];
            if (sensorsValues[i] > sensorsMaxValues[i])
                sensorsMaxValues[i] = sensorsValues[i];
        }
    }
    stop();
}

template <size_t S>
short LineFollower<S>::getInicialSpeed() {
    return m_initialSpeed;
}

template <size_t S>
byte LineFollower<S>::readSensors() {
    // Black: True
    // White: False
    byte result = 0, i;
    for (i = 0; i < S; i++) {
        sensorsValues[i] = digitalRead(sensorsPins[i]);
        sensorsInBlack[i] = !sensorsValues[i];
        // sensorsInBlack[i] = !sensorsValues[i];
        //     sensorsValues[i] == sensorsMaxValues[i] ? false :
        //     sensorsValues[i] == sensorsMinValues[i] ? true : 
        //     sensorsValues[i] < (sensorsMaxValues[i] + sensorsMinValues[i])/2;
        result |= sensorsInBlack[i] << S - i - 1;
    }
    return result;
}