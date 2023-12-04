#include "MazeSolver.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 80

// Sensor Configuration
#define LEFT_OUTER_SENSOR_PIN 11
#define LEFT_INNER_SENSOR_PIN 12
#define RIGHT_INNER_SENSOR_PIN 4
#define RIGHT_OUTER_SENSOR_PIN 3

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PIN_1 5
#define LEFT_MOTOR_PIN_2 6
#define RIGHT_MOTOR_PIN_1 9
#define RIGHT_MOTOR_PIN_2 10

// Constants Configuration
// PID
#define KP 80 * ((INITIAL_SPEED+50)/100)
#define KI 0
#define KD 160

// Error Weights
#define CENTRAL_SENSOR_ERROR 0
#define INNER_SENSORS_ERROR 2
#define OUTER_SENSORS_ERROR 4

#define NUMBER_OF_SENSORS 2
#define MAX_PATH_LENGTH 100

MazeSolver *ms = new MazeSolver(INITIAL_SPEED, MAX_SPEED, NUMBER_OF_SENSORS, MAX_PATH_LENGTH);
byte sensorsPins[] = {LEFT_INNER_SENSOR_PIN, RIGHT_INNER_SENSOR_PIN};

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando Configuração");
    pinMode(LED_BUILTIN, OUTPUT);
    ms->setConstants(KP, KI, KD);
    ms->setErrorWeights(INNER_SENSORS_ERROR, OUTER_SENSORS_ERROR, CENTRAL_SENSOR_ERROR);
    ms->setMotorsPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
    ms->setSensorsPins(sensorsPins);
    ms->setSharpTurnSensors(LEFT_OUTER_SENSOR_PIN, RIGHT_OUTER_SENSOR_PIN);
    // ms->calibrateSensors();
}

void loop() {
   ms->solver();
    // if (digitalRead(LEFT_OUTER_SENSOR_PIN))
    //     ms->forward();
    // else ms->stop();
}