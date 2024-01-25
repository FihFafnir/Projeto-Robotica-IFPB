#include "MazeSolver.h"

// Speed Configuration
#define MAX_SPEED 255
#define INITIAL_SPEED 80

// Sensor Configuration
#define LEFT_OUTER_SENSOR_PIN 13
#define LEFT_INNER_SENSOR_PIN 12
#define CENTRAL_SENSOR_PIN 11
#define RIGHT_INNER_SENSOR_PIN 4
#define RIGHT_OUTER_SENSOR_PIN 3

#define SHARP_LEFT_TURN_SENSOR_PIN 8
#define SHARP_RIGHT_TURN_SENSOR_PIN 7

// Motors Configuration
// Use PWM Pins (Arduino Uno = 3, 5, 6, 9, 10, 11)
#define LEFT_MOTOR_PIN_1 10
#define LEFT_MOTOR_PIN_2 9
#define RIGHT_MOTOR_PIN_1 6
#define RIGHT_MOTOR_PIN_2 5

// Constants Configuration
// PID
#define KP 80
#define KI 0
#define KD 160

// Error Weights
#define CENTRAL_SENSOR_ERROR 0
#define INNER_SENSORS_ERROR 2
#define OUTER_SENSORS_ERROR 4

#define NUMBER_OF_SENSORS 5
#define MAX_PATH_LENGTH 100

MazeSolver<NUMBER_OF_SENSORS> ms(INITIAL_SPEED, MAX_SPEED);
byte sensorsPins[] = {
    LEFT_OUTER_SENSOR_PIN, 
    LEFT_INNER_SENSOR_PIN, 
    CENTRAL_SENSOR_PIN, 
    RIGHT_INNER_SENSOR_PIN, 
    RIGHT_OUTER_SENSOR_PIN
};

void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando Configuração");
    ms.setConstants(KP, KI, KD);
    ms.setErrorWeights(INNER_SENSORS_ERROR, OUTER_SENSORS_ERROR, CENTRAL_SENSOR_ERROR);
    ms.setMotorsPins(LEFT_MOTOR_PIN_1, LEFT_MOTOR_PIN_2, RIGHT_MOTOR_PIN_1, RIGHT_MOTOR_PIN_2);
    ms.setSensorsPins(sensorsPins);
    ms.setSharpTurnSensors(SHARP_LEFT_TURN_SENSOR_PIN, SHARP_RIGHT_TURN_SENSOR_PIN);
    // ms.calibrateSensors();
}

void loop() {
    ms.solver();
}