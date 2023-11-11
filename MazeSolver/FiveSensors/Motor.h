#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED
typedef unsigned char byte;

class Motor {
    byte maxSpeed, pins[2];
    short speed;
    public:
        static const byte 
            NORMAL = 1,
            REVERSE = 2;
        Motor(byte firstPin, byte secondPin, byte maxSpeed);
        void rotate(byte firstPin, byte secondPin);
        void rotate(byte direction);
        void stop();
        void setSpeed(short newSpeed);
        short getSpeed();
};

Motor::Motor(byte firstPin, byte secondPin, byte maxSpeed) {
    this->pins[0] = firstPin;
    this->pins[1] = secondPin;
    this->maxSpeed = maxSpeed;
    pinMode(firstPin, OUTPUT);
    pinMode(secondPin, OUTPUT);
}

void Motor::rotate(byte firstPin, byte secondPin) {
    analogWrite(firstPin, abs(speed));
    digitalWrite(secondPin, LOW);
}

void Motor::rotate(byte direction) {
    rotate(pins[direction >> 1], pins[direction & 1]);
}

void Motor::stop() {
    digitalWrite(pins[0], LOW);
    digitalWrite(pins[1], LOW);
}

void Motor::setSpeed(short newSpeed) {
    speed = min(max(-maxSpeed, newSpeed), maxSpeed);
}

short Motor::getSpeed() {
    return speed;
}

#endif
