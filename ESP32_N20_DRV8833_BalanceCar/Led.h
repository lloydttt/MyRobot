#pragma once

/*注意：此库 适合LED低电平点亮的开发板*/

#include <Arduino.h>

class Led
{
public:
    Led(int pin);         // 构造函数
    int ledPin;           // LED引脚
    void on();            // 灯亮，默认最大亮度（pwm255）
    void on(int);         // 灯亮，延时后灭
    void onPwm(int);      // 设定亮灯的pwm
    void onPwm(int, int); // 以一定pwm亮灯，延时一定时间后熄灭
    void off();           // 灯灭
};

Led::Led(int pin)
{
    ledPin = pin;             // 构造函数初始化时指定led 引脚
    analogWrite(ledPin, 255); // 初始化时灯灭
}

/*注意，LOLIN32 Lite开发板板载led为22引脚，是低电平为点亮，高电平熄灭*/
void Led::on()
{
    analogWrite(ledPin, 0);
}

void Led::on(int time)
{
    analogWrite(ledPin, 0);
    delay(time); // 灯亮延时时长，之后熄灭
    analogWrite(ledPin, 255);
}

void Led::onPwm(int pwmval)
{
    analogWrite(ledPin, 255 - pwmval);
}

void Led::onPwm(int pwmval, int time)
{
    analogWrite(ledPin, 255 - pwmval);
    delay(time); // 灯亮延时时长，之后熄灭
    analogWrite(ledPin, 255);
}

void Led::off()
{
    analogWrite(ledPin, 255);
}
