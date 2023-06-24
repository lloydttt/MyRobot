#pragma once

#include <Arduino.h>

class Motor
{
public:
    int pinA, pinB, pwm;
    Motor(int a, int b);
    void run(int pwm); // 运动
    void brake();      // 刹车
    void flameout();   // 熄火
};

Motor::Motor(int a, int b)
{
    pinA = a;
    pinB = b;
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
}

void Motor::run(int pwm)
{
    if (pwm >= 0)
    {
        analogWrite(pinA, pwm);
        analogWrite(pinB, 0);
    }
    else
    {
        analogWrite(pinA, 0);
        analogWrite(pinB, -pwm);
    }
}

void Motor::brake()
{
    analogWrite(pinA, 255);
    analogWrite(pinB, 255);
}

void Motor::flameout()
{
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
}
