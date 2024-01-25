#pragma once
#define MAX_SPEED_DEFAULT 255
template <typename T>
class Pair {
    T m_data[2];

    public:
        T& first = m_data[0];
        T& second = m_data[1];

        Pair();
        Pair(T first, T second);
        T& operator[] (size_t index);
};

template <typename T>
Pair<T>::Pair() {}

template <typename T>
Pair<T>::Pair(T first, T second) {
    this->first = first;
    this->second = second;
}

template <typename T>
T& Pair<T>::operator[](size_t index) {
    return m_data[index];
}

class Motor {
    Pair<byte> m_pins;
    byte m_maxSpeed;
    short m_speed;

    public:
        static const byte 
            NORMAL = 1,
            REVERSE = 2;

        Motor();
        Motor(byte firstPin, byte secondPin);
        Motor(byte firstPin, byte secondPin, byte maxSpeed);

        void setPins(byte firstPin, byte secondPin);

        void rotate(byte firstPin, byte secondPin);
        void rotate(byte direction);
        void rotate();
        void stop();
        
        void setSpeed(short speed);
        short getSpeed();
};

Motor::Motor() : m_maxSpeed(MAX_SPEED_DEFAULT) {}

Motor::Motor(byte firstPin, byte secondPin) : m_maxSpeed(MAX_SPEED_DEFAULT) {
    setPins(firstPin, secondPin);
}

Motor::Motor(byte firstPin, byte secondPin, byte maxSpeed) : m_maxSpeed(maxSpeed) {
    setPins(firstPin, secondPin);
}

void Motor::setPins(byte firstPin, byte secondPin) {
    m_pins.first = firstPin;
    m_pins.second = secondPin;

    pinMode(firstPin, OUTPUT);
    pinMode(secondPin, OUTPUT);
}

void Motor::rotate(byte firstPin, byte secondPin) {
    analogWrite(firstPin, abs(m_speed));
    digitalWrite(secondPin, LOW);
}

void Motor::rotate(byte direction) {
    rotate(m_pins[direction >> 1], m_pins[direction & 1]);
}

void Motor::rotate() {
    rotate(m_speed > 0 ? NORMAL : REVERSE);
}

void Motor::stop() {
    digitalWrite(m_pins.first, LOW);
    digitalWrite(m_pins.second, LOW);
}

void Motor::setSpeed(short speed) {
    m_speed = constrain(speed, -m_maxSpeed, m_maxSpeed);
}

short Motor::getSpeed() {
    return m_speed;
}