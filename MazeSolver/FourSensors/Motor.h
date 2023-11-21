#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED

class Motor {
    byte maxSpeed, pins[4];
    short speed;
    bool enInput;
    public:
        static const byte 
            NORMAL = 1,
            REVERSE = 2;
        Motor(byte firstPin, byte secondPin, byte maxSpeed);
        Motor(byte firstPin, byte secondPin, byte enPin, byte maxSpeed);
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
    this->enInput = false;
    pinMode(firstPin, OUTPUT);
    pinMode(secondPin, OUTPUT);
}

Motor::Motor(byte firstPin, byte secondPin, byte enPin, byte maxSpeed) {
    this->pins[0] = firstPin;
    this->pins[1] = secondPin;
    this->pins[2] = enPin;
    this->maxSpeed = maxSpeed;
    this->enInput = true;
    pinMode(firstPin, OUTPUT);
    pinMode(secondPin, OUTPUT);
}

void Motor::rotate(byte firstPin, byte secondPin) {
    digitalWrite(secondPin, LOW);
    if (enInput) {
        analogWrite(pins[2], abs(speed));
        digitalWrite(firstPin, HIGH);
    } else analogWrite(firstPin, abs(speed));
}

void Motor::rotate(byte direction) {
    rotate(pins[direction >> 1], pins[direction & 1]);
}

void Motor::stop() {
    digitalWrite(pins[0], LOW);
    digitalWrite(pins[1], LOW);
}

void Motor::setSpeed(short newSpeed) {
    speed = constrain(newSpeed, -maxSpeed, maxSpeed);
}

short Motor::getSpeed() {
    return speed;
}

#endif
