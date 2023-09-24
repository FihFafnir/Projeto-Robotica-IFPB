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

typedef unsigned short ushort;

Vehicle *myV = new Vehicle(CURRENT_SPEED, MAX_SPEED);

int readSensors(int leftMostSensorPin, int leftSensorPin, int rightSensorPin, int rightMostSensorPin) {
    bool 
        leftMostSensorValue = digitalRead(leftMostSensorPin) != HIGH,
        leftSensorValue = digitalRead(leftSensorPin) != HIGH,
        rightSensorValue = digitalRead(rightSensorPin) != HIGH,
        rightMostSensorValue = digitalRead(rightMostSensorPin) != HIGH;

    return leftMostSensorValue << 3 | leftSensorValue << 2 | rightSensorValue << 1 | rightMostSensorValue;
}

void setup() {
    ushort pins[] = {RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PIN_1};
    myV->setPins(pins);
    pinMode(LEFTMOST_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(RIGHTMOST_SENSOR_PIN, INPUT);
}

void loop() {
    switch (readSensors(LEFTMOST_SENSOR_PIN, LEFT_SENSOR_PIN, RIGHT_SENSOR_PIN, RIGHTMOST_SENSOR_PIN)) {
        case 0: // 0000
            // 180°
            myV->rotateRight();
            delay(600);
            break;
        case 2: // 0010
            myV->rotateRight();
            break;
        case 4: // 0100
            myV->rotateLeft();
            break;
        case 6: // 0110
            myV->forward();
            break;
        case 7: // 0111
            // 90° Right
            myV->rotateRight();
            delay(300);
        case 14: // 1110
            // 90° Left
            myV->rotateLeft();
            delay(300);
        default:
            myV->stop();
    }
}
