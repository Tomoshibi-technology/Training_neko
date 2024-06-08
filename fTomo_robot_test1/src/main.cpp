#include <Arduino.h>

class LED{
  private:
    int PIN;
    bool state;
  public:
    LED(int ptr_pin);
    void init();
    void light();
    void off();
};


LED::LED(int ptr_pin){
  PIN = ptr_pin;
}

void LED::init(){
  pinMode(PIN, OUTPUT);
}

void LED::light(){
  digitalWrite(PIN, HIGH);
}

void LED::off(){
  digitalWrite(PIN, LOW);
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
    led[i].light();
  }
  delay(200);
  for(int i = 0; i < 2; i++){
    led[i].off();
  }
  delay(200);
}
