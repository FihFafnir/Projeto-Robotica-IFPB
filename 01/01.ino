#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define CURRENT_SPEED 80

// Sensor Configuration
#define LEFT_SENSOR_PIN 12
#define RIGHT_SENSOR_PIN 7

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 10, 11)
#define LEFT_MOTOR_PIN_1 11
#define LEFT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

typedef unsigned short ushort;

Vehicle *myV = new Vehicle(CURRENT_SPEED, MAX_SPEED);

void setup() {
    ushort pins[] = {RIGHT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, LEFT_MOTOR_PIN_1};
    myV->setPins(pins);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    if (digitalRead(LEFT_SENSOR_PIN) && digitalRead(RIGHT_SENSOR_PIN))
        myV->stop();
    else if (digitalRead(LEFT_SENSOR_PIN))
        myV->rotateLeft();
    else if (digitalRead(RIGHT_SENSOR_PIN))
        myV->rotateRight();
    else myV->forward();
}
