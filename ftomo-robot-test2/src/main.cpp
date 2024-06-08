#include <Arduino.h>

int M0A = 10;
int M0B = 11;
int M1A = 12;
int M1B = 13;
int M2A = 18;
int M2B = 19;
int M3A = 16;
int M3B = 17;
int strtSW_pin = 14;

int strtSW = 0;
int go_dig = 0;
int go_sp = 0;
int n = 0;

int drvDigSp(int mtrNum, int moveDig, int moveSp) {
  moveDig = (moveDig + 180) % 360 - 180;
  int drv = sin((moveDig + (135.0 - mtrNum * 90.0)) / 180.0 * 3.14) * moveSp;
  return drv;
}

void drvMotor(int pinA, int pinB, int mtrSp) {
  if(mtrSp > 0) {
    analogWrite(pinA, mtrSp);
    analogWrite(pinB, 0);
  }else if(mtrSp < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, abs(mtrSp));
  }else{
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void move(int mtrNum, int pinA, int pinB, int moveDig, int moveSp){
  int mtrSp = drvDigSp(mtrNum, moveDig, moveSp);
  drvMotor(pinA, pinB, mtrSp);
}

//--------------------------------------------------------------------------------------
void setup() {
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(strtSW_pin, INPUT);

  Serial.begin(115200);
}

void loop() {
  n++;
  go_dig = n / 100;
  go_sp = 50;

  strtSW = digitalRead(strtSW_pin);
  if(strtSW == 0){
    go_sp = go_sp;
  }else{
    go_sp = 0;
  }

  move(0, M0A, M0B, go_dig, go_sp);
  move(1, M1A, M1B, go_dig, go_sp);
  move(2, M2A, M2B, go_dig, go_sp);
  move(3, M3A, M3B, go_dig, go_sp);
}
