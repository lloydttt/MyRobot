#include <Arduino.h>
#include <TFT_eSPI.h>
#include <Wire.h>

TFT_eSPI tft = TFT_eSPI();

const uint16_t bmp1 [] PROGMEM = {};


void setup() {
  tft.init();
  tft.setSwapBytes(true);
  tft.fillScreen(TFT_BLACK);
}
 
void loop() {
  tft.pushImage(0,0,240,240,bmp1);
}
