#include <Arduino.h>

class LED{
  private:
    int PIN;
    bool state = 0;
  public:
    LED(int ptr_pin);
    void init();
    void light();
    void off();
    void toggle();
};


LED::LED(int ptr_pin){
  PIN = ptr_pin;
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

//--------------------------

// LED led(21);
LED led[2] = {
  LED(21),
  LED(22)
};

void setup(){
  for(int i = 0; i < 2; i++){
    led[i].init();
  }
}

void loop(){
  for(int i = 0; i < 2; i++){
    led[i].toggle();
  }
  delay(500);
}
