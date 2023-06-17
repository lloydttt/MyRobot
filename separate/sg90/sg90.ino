#include <Arduino.h>

#define sgpwm 23
int freq = 50;
int channel = 8;
int resolution = 8;

int calculatePWM(int degree){
  const float deadZone = 6.4;
  const float maxi = 32;
  if(degree<0){
    degree = -degree;
  }
  if(degree>180)
    degree = 180;
  return (int)(((maxi-deadZone)/180)*degree+deadZone);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcSetup(channel,freq,resolution);
  ledcAttachPin(sgpwm,channel);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i =0;i<=180;i+=10)
  {
    ledcWrite(channel,calculatePWM(i));
    Serial.println("turn");
    delay(1000);
  }
  delay(1000);
  Serial.println("back");
  for(int i =180;i>=0;i-=10)
  {
    ledcWrite(channel,calculatePWM(i));
    Serial.println("turn");
    delay(1000);
  }
  
}
