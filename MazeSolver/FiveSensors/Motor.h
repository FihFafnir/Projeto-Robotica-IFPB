#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED
typedef unsigned char byte;

class Motor {
    byte speed, maxSpeed;
    byte pins[2];
    public:
        Motor(byte firstPin, byte secondPin, byte maxSpeed);
        byte getSpeed();
        void setSpeed(byte value);
        void rotateClockwise();
        void rotateCounterclockwise();
        void stop();
    private:
        void rotate(byte firstPin, byte secondPin);
};

Motor::Motor(byte firstPin, byte secondPin, byte maxSpeed) {
    pins[0] = firstPin;
    pins[1] = secondPin;
    maxSpeed = maxSpeed;
    pinMode(firstPin, OUTPUT);
    pinMode(secondPin, OUTPUT);
}

byte Motor::getSpeed() {
    return speed;
}

void Motor::setSpeed(byte value) {
    speed = max(min(value, maxSpeed), 0);
}

void Motor::rotate(byte firstPin, byte secondPin) {
    analogWrite(firstPin, speed);
    digitalWrite(secondPin, LOW);
}

void Motor::rotateClockwise() {
    rotate(pins[1], pins[0]);
}

void Motor::rotateCounterclockwise() {
    rotate(pins[0], pins[1]);
}

void Motor::stop() {
    digitalWrite(pins[0], LOW);
    digitalWrite(pins[1], LOW);
}

#endif
