
#include <SPI.h>
#include <ADXL345.h>
ADXL345 test(10);
void setup() {
  
  // put your setup code here, to run once:
 Serial.begin(115200);
 test.begin();
 delay(500);

 Serial.println(test.getDeviceID(),BIN);
}

void loop() {
  // put your main code here, to run repeatedly:
 Serial.println(test.getX(),DEC);
 analogWrite(3,map(abs(test.getX()),0,800,0,255));
 delay(5);
}
