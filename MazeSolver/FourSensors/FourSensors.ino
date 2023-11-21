#include "MazeSolver.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 100

// Sensor Configuration
#define LEFT_OUTER_SENSOR_PIN A5
#define LEFT_INNER_SENSOR_PIN A4
#define RIGHT_INNER_SENSOR_PIN A3
#define RIGHT_OUTER_SENSOR_PIN A2

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_EN_PIN 5
#define LEFT_MOTOR_PIN_1 7
#define LEFT_MOTOR_PIN_2 6
#define RIGHT_MOTOR_PIN_1 8
#define RIGHT_MOTOR_PIN_2 9
#define RIGHT_EN_PIN 10

// Constants Configuration
// PID
#define KP 60 * ((INITIAL_SPEED+50)/100)
#define KI 0
#define KD 160

// Error Weights
#define CENTRAL_SENSOR_ERROR 0
#define INNER_SENSORS_ERROR 2
#define OUTER_SENSORS_ERROR 4

#define NUMBER_OF_SENSORS 4
#define MAX_PATH_LENGTH 100

MazeSolver *ms = new MazeSolver(INITIAL_SPEED, MAX_SPEED, NUMBER_OF_SENSORS, MAX_PATH_LENGTH);
byte sensorsPins[] = {LEFT_OUTER_SENSOR_PIN, LEFT_INNER_SENSOR_PIN, RIGHT_INNER_SENSOR_PIN, RIGHT_OUTER_SENSOR_PIN};

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando Configuração");
    ms->setConstants(KP, KI, KD);
    ms->setErrorWeights(INNER_SENSORS_ERROR, OUTER_SENSORS_ERROR, CENTRAL_SENSOR_ERROR);
    ms->setMotorsPins(LEFT_EN_PIN, LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2, RIGHT_EN_PIN);
    ms->setSensorsPins(sensorsPins);
    // ms->calibrateSensors();
}

void loop() {
    if (
        digitalRead(LEFT_OUTER_SENSOR_PIN) &&
        digitalRead(LEFT_INNER_SENSOR_PIN) &&
        digitalRead(RIGHT_INNER_SENSOR_PIN) &&
        digitalRead(RIGHT_OUTER_SENSOR_PIN)
    ) ms->stop();
    else ms->solver();
}