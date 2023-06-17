#include <Arduino.h>
#define L_Front 12
#define L_Back 13
#define R_Front 14
#define R_Back 26

const int freq = 20000;
const int resolution = 8;

void setup() {
  // put your setup code here, to run once:
  pinMode(L_Front,OUTPUT);
  pinMode(L_Back,OUTPUT);
  pinMode(R_Front,OUTPUT);
  pinMode(R_Back,OUTPUT);
  ledcSetup(0,freq,resolution);
  ledcSetup(1,freq,resolution);
  ledcSetup(2,freq,resolution);
  ledcSetup(3,freq,resolution);
  ledcAttachPin(L_Front,0);
  ledcAttachPin(L_Back,1);
  ledcAttachPin(R_Front,2);
  ledcAttachPin(R_Back,3);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  wheelControl(true,true,128);
  delay(1500);
  wheelControl(false,false,128);
  delay(1500);
}

void wheelControl(bool l_D, bool r_D, int speedi) {
  if(l_D&&r_D){
    ledcWrite(0,speedi);
    ledcWrite(2,speedi);
  }
  else if(!l_D&&!r_D){
    ledcWrite(1,speedi);
    ledcWrite(3,speedi);
  }
  else if(l_D&&!r_D){     //右转
    ledcWrite(0,speedi);
    ledcWrite(3,speedi);
  }
  else if(!l_D&&r_D){     //左转
    ledcWrite(1,speedi);
    ledcWrite(2,speedi);
  }
  
}
