#include <Arduino.h>
#include "./drvmotor/drvmotor.h"
#include "./calmotor/calmotor.h"

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


// int drvDigSp(int mtrNum, int moveDig, int moveSp);
// void move(int mtrNum, int pinA, int pinB, int moveDig, int moveSp);
//--------------------------

DRVMOTOR drv[4] = {
  DRVMOTOR(M0A, M0B), DRVMOTOR(M1A, M1B), DRVMOTOR(M2A, M2B), DRVMOTOR(M3A, M3B)
};
// DRVMOTOR drv0(M0A, M0B);

CALMOTOR cal(drv);

void setup() {
  pinMode(21, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(strtSW_pin, INPUT);

  Serial.begin(115200);

  cal.init();
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

  cal.setGo(go_sp, go_dig);
}


//--------------------------------------------------------------------------------------

// int drvDigSp(int mtrNum, int moveDig, int moveSp) {
//   moveDig = (moveDig + 180) % 360 - 180;
//   int sp = sin((moveDig + (135.0 - mtrNum * 90.0)) / 180.0 * 3.14) * moveSp;
//   return sp;
// }

// void move(int mtrNum, int pinA, int pinB, int moveDig, int moveSp){
//   int mtrSp = drvDigSp(mtrNum, moveDig, moveSp);
//   DRVMOTOR(pinA, pinB, mtrSp);
// }