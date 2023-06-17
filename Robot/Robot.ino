#include <Arduino.h>
#include "My_IMU.h"
#include "My_encoder.h"

int16_t Velocity_Left, Velocity_Right;     //左右轮速度

void setup() {
  // put your setup code here, to run once:
  Initial_IMU();
  Initial_encoder();
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU_run();
  encoder_run();
}
