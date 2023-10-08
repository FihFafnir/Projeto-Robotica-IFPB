#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 80

// Sensor Configuration
#define LEFT_SENSOR_PIN 12
#define RIGHT_SENSOR_PIN 7

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 10, 11)
#define LEFT_MOTOR_PIN_1 11
#define LEFT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 25
#define KI 0
#define KD 15

// Error Weights Configuration
#define LEFT_SENSOR_ERROR 1
#define RIGHT_SENSOR_ERROR 1

typedef unsigned char byte;

Vehicle *myV = new Vehicle(INITIAL_SPEED, MAX_SPEED);
int p = 0, i = 0, d = 0, pid = 0, currentError = 0, previousError = 0;


int calculateError(int leftSensorPin, int rightSensorPin) {
    bool 
        leftSensorValue = digitalRead(leftSensorPin),
        rightSensorValue = digitalRead(rightSensorPin);
 
    return !leftSensorValue * LEFT_SENSOR_ERROR - !rightSensorValue * RIGHT_SENSOR_ERROR;
}

int calculatePID() {
    p = currentError, 
    d = currentError - previousError;
    i = currentError ? i + currentError : 0;
    return KP*p + KI*i + KD*d;
}

void setup() {
    byte pins[] = {RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PIN_1};
    myV->setPins(pins);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    currentError = calculateError(LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN);
    pid = calculatePID();
    previousError = currentError;
    myV->setSpeed(INITIAL_SPEED + pid, INITIAL_SPEED - pid);

    if (digitalRead(LEFT_SENSOR_PIN) && digitalRead(RIGHT_SENSOR_PIN)) {
        myV->stop();
        myV->setSpeed(INITIAL_SPEED);
        currentError = previousError = pid = 0;
    } else myV->forward();
}
