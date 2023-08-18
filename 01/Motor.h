#ifndef MOTOR_H_INCLUDED
#define MOTOR_H_INCLUDED
typedef unsigned short ushort;

class Motor {
    ushort speed, maxSpeed;
    ushort pins[2];
    public:
        Motor(ushort firstPin, ushort secondPin, ushort speed, ushort maxSpeed);
        void setSpeed(ushort value);
        void rotateClockwise();
        void rotateCounterclockwise();
        void stop();
    private:
        void rotate(ushort firstPin, ushort secondPin);
};

Motor::Motor(ushort firstPin, ushort secondPin, ushort spd, ushort maxSpd) {
    pins[0] = firstPin;
    pins[1] = secondPin;
    speed = spd;
    maxSpeed = maxSpd;
}

void Motor::setSpeed(ushort value) {
    speed = min(value, maxSpeed);
}

void Motor::rotate(ushort firstPin, ushort secondPin) {
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
