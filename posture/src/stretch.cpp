#include <Arduino.h>

const int stretchPin = 36;

void setup() {
  Serial.begin(9600);
  pinMode(stretchPin, INPUT);
}

void loop() {
  int value = analogRead(stretchPin);    
  Serial.printf(">%d", value);  
  delay(10); 
}