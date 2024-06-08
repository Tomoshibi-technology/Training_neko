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

int drvDigSp(int mtrNum, int dig, int sp) {
  int drv = sin((dig + (135.0 - mtrNum * 90.0)) / 180.0 * 3.14) * sp;
  return drv;
}

void drvMotor(int pinA, int pinB, int sp) {
  if(sp > 0) {
    analogWrite(pinA, sp);
    analogWrite(pinB, 0);
  } else if (sp < 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, abs(sp));
  } else {
    analogWrite(pinA, 0);
    analogWrite(pinB, 0);
  }
}

void setup() {
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(strtSW_pin, INPUT);

  Serial.begin(115200);
}

void loop() {
  strtSW = digitalRead(strtSW_pin);
  
  go_dig = 0;
  go_sp = 100;

  int sp0 = drvDigSp(0, go_dig, go_sp);
  int sp1 = drvDigSp(1, go_dig, go_sp);
  int sp2 = drvDigSp(2, go_dig, go_sp);
  int sp3 = drvDigSp(3, go_dig, go_sp);

  if(strtSW == 1){
    sp0 = sp0;
    sp1 = sp1;
    sp2 = sp2;
    sp3 = sp3;
  } else {
    sp0 = sp1 = sp2 = sp3 = 0;
  }

  drvMotor(M0A, M0B, sp0);
  drvMotor(M1A, M1B, sp1);
  drvMotor(M2A, M2B, sp2);
  drvMotor(M3A, M3B, sp3);

}
