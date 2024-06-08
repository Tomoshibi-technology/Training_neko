#include "./led.h"

LED::LED(int ptr_pin){ //つくられるときに一回だけ実行される
  PIN = ptr_pin; 
  state = 0;
}

void LED::init(){
  pinMode(PIN, OUTPUT);
}

void LED::light(){
  state = 1;
  digitalWrite(PIN, HIGH);
}

void LED::off(){
  state = 0;
  digitalWrite(PIN, LOW);
}

void LED::toggle(){
  state = !state;
  digitalWrite(PIN, state);
}