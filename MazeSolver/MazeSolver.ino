#include "Vehicle.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 70

// Sensor Configuration
#define LEFTMOST_SENSOR_PIN 12
#define LEFT_SENSOR_PIN 11
#define RIGHT_SENSOR_PIN 4
#define RIGHTMOST_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PIN_1 9
#define LEFT_MOTOR_PIN_2 10
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 30
#define KI 0.00001
#define KD 60
// Error Weights
#define MIDDLE_SENSORS_ERROR 1
#define OUTER_SENSORS_ERROR 2.2

typedef unsigned char byte;

Vehicle *myV = new Vehicle(INITIAL_SPEED, MAX_SPEED);
float p = 0, i = 0, d = 0, pid = 0, currentError = 0, previousError = 0;
char path[100];
int pathLength = 0;
bool end = true;

int calculateError() {
    bool 
        leftMostSensorValue = digitalRead(LEFTMOST_SENSOR_PIN),
        leftSensorValue = digitalRead(LEFT_SENSOR_PIN),
        rightSensorValue = digitalRead(RIGHT_SENSOR_PIN),
        rightMostSensorValue = digitalRead(RIGHTMOST_SENSOR_PIN);

    if (leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 100;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && rightMostSensorValue)
        return 101;
    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return 102;
    if (leftMostSensorValue && !leftSensorValue && !rightSensorValue && rightMostSensorValue)
        return 103;

    if (!leftMostSensorValue && !leftSensorValue && !rightSensorValue && rightMostSensorValue)
        return OUTER_SENSORS_ERROR;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && rightMostSensorValue)
        return (OUTER_SENSORS_ERROR + MIDDLE_SENSORS_ERROR)/2;
    if (!leftMostSensorValue && !leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return MIDDLE_SENSORS_ERROR;
    if (!leftMostSensorValue && leftSensorValue && rightSensorValue && !rightMostSensorValue)
        return 0;
    if (!leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -MIDDLE_SENSORS_ERROR;
    if (leftMostSensorValue && leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -(OUTER_SENSORS_ERROR + MIDDLE_SENSORS_ERROR)/2;
    if (leftMostSensorValue && !leftSensorValue && !rightSensorValue && !rightMostSensorValue)
        return -OUTER_SENSORS_ERROR;
    // return (rightMostSensorValue*OUTER_SENSORS_ERROR + rightSensorValue*MIDDLE_SENSORS_ERROR)/max(1, rightMostSensorValue+rightSensorValue)
        //  - (leftMostSensorValue*OUTER_SENSORS_ERROR + leftSensorValue*MIDDLE_SENSORS_ERROR)/max(1, leftMostSensorValue+leftSensorValue);
}

int calculatePID() {
    currentError = calculateError();
    p = currentError, 
    d = currentError - previousError;
    i += currentError;
    previousError = currentError;
    return KP*p + KI*i + KD*d;
}

void followLine() {
    pid = calculatePID();
    myV->setSpeed(INITIAL_SPEED + pid, INITIAL_SPEED - pid);
    myV->forward();
}

void setup() {
    Serial.begin(9600);
    byte pins[] = {LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2};
    myV->setPins(pins);
    pinMode(LEFTMOST_SENSOR_PIN, INPUT);
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);
    pinMode(RIGHTMOST_SENSOR_PIN, INPUT);
}

void loop() {
    if (end) {
        myV->stop();
        myV->setSpeed(INITIAL_SPEED);
        previousError = pid = p = i = d = 0;
        currentError = calculateError();
        if (currentError == 0)
            end = false;
    } else switch(round(currentError)) {
        case 100: // Turn to left (90°)
            myV->setSpeed(INITIAL_SPEED);
            myV->forward();
            do currentError = calculateError();
            while (currentError == 100);
            delay(400);

            if (!(pathLength >= 2 && path[pathLength-1] == 'B'&& path[pathLength-2] == 'R')) {
                if (currentError == 102 || (path[pathLength-1] == 'B' && path[pathLength-2] == 'S')) {
                    do {
                        myV->rotateLeft();
                        currentError = calculateError();
                    } while (currentError < 100);
                    do {
                        myV->rotateLeft();
                        currentError = calculateError();
                    } while (currentError != 0);
                    path[pathLength++] = 'L';
                } else path[pathLength++] = 'S';

                Serial.println(path);
                Serial.println(pathLength);
            }
            break;

        case 101: // Turn to right (90°)
            myV->setSpeed(INITIAL_SPEED);
            myV->forward();
            do currentError = calculateError();
            while (currentError == 101);
            delay(400);

            if (!(pathLength >= 2 && path[pathLength-1] == 'B' && path[pathLength-2] == 'L')) {
                if (currentError == 102 || (path[pathLength-1] == 'B' && path[pathLength-2] == 'S')) {
                    do {
                        myV->rotateRight();
                        currentError = calculateError();
                    } while (currentError < 100);
                    do {
                        myV->rotateRight();
                        currentError = calculateError();
                    } while (currentError != 0);
                    path[pathLength++] = 'R';
                } else path[pathLength++] = 'S';

                Serial.println(path);
                Serial.println(pathLength);
            }
            break;

        case 102: // Make U (180°)
            path[pathLength++] = 'B';
            Serial.println(path);
            Serial.println(pathLength);
            myV->setSpeed(INITIAL_SPEED);
            do {
                myV->rotateRight();
                currentError = calculateError();
            } while (currentError != 0);
            break;

        case 103: // Stop
            end = true;
            break;
        default:
            followLine();
    }
}
