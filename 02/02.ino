#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define CURRENT_SPEED 80

// Sensor Configuration
#define LEFTMOST_SENSOR_PIN 13
#define LEFT_SENSOR_PIN 12
#define RIGHT_SENSOR_PIN 4
#define RIGHTMOST_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 10, 11)
#define LEFT_MOTOR_PIN_1 11
#define LEFT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 0
#define KI 35
#define KD 35

// Error Weights Configuration
#define LEFT_MOST_SENSOR_ERROR -1.5
#define LEFT_SENSOR_ERROR -1
#define RIGHT_SENSOR_ERROR 1
#define RIGHT_MOST_SENSOR_ERROR 1.5

typedef unsigned char byte;

Vehicle *myV = new Vehicle(CURRENT_SPEED, MAX_SPEED);
int p = 0, i = 0, d = 0, pid = 0, currentError = 0, previousError = 0;

int readSensors(int leftMostSensorPin, int leftSensorPin, int rightSensorPin, int rightMostSensorPin) {
    bool 
        leftMostSensorValue = digitalRead(leftMostSensorPin) != HIGH,
        leftSensorValue = digitalRead(leftSensorPin) != HIGH,
        rightSensorValue = digitalRead(rightSensorPin) != HIGH,
        rightMostSensorValue = digitalRead(rightMostSensorPin) != HIGH;

    return leftMostSensorValue*LEFT_MOST_SENSOR_ERROR + leftSensorValue*LEFT_SENSOR_ERROR + rightSensorValue*RIGHT_SENSOR_ERROR + rightMostSensorValue*RIGHT_MOST_SENSOR_ERROR;
}

int calculatePID() {
    currentError = readSensors(LEFT_MOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHT_MOST_SENSOR_PIN);
    p = currentError, 
    d = currentError - previousError;
    i = currentError ? i + currentError : 0;
    if (i > 255)
        i = 255;
    else if(i < -255)
        i = -255;
    previousError = currentError;
    return KP*p + KI*i + KD*d;
}

void setup() {
    byte pins[] = {RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PIN_1};
    myV->setPins(pins);
    pinMode(LEFTMOST_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(RIGHTMOST_SENSOR_PIN, INPUT);
}

void loop() {
    pid = calculatePID();
    if (pid >= 0) 
        myV->setSpeed(myV->getLeftMotorSpeed(), myV->getRightMotorSpeed() - pid);
    else 
        myV->setSpeed(myV->getLeftMotorSpeed() + pid, myV->getRightMotorSpeed());
}
