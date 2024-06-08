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
    void toggle();
};
//publicは外からアクセスできる。クラスの中でしか使えない。クラスの外からはクラス名.でアクセスできる。変数も同じ

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

//--------------------------

// LED led(21);

LED led[2] = { //小文字ledはクラス名。大文字LEDはクラス名
  LED(21),
  LED(22)
};

void setup(){
  for(int i = 0; i < 2; i++){
    led[i].init();
  }
  led[0].light();
}

void loop(){
  for(int i = 0; i < 2; i++){
    led[i].toggle();
  }
  delay(500);
}
