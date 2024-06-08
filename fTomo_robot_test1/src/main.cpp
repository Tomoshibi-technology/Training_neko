#include <Arduino.h>
#include "./led/led.h" //クラスのヘッダファイルをインクルード

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
