#include <Arduino.h>

void setup() {
  pinMode(21, OUTPUT);
}

void loop() {
  digitalWrite(21, HIGH);
  delay(500);
}

