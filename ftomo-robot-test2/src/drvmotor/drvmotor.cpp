#include "./drvmotor.h"

DRVMOTOR::DRVMOTOR(int ptr_A, int ptr_B){
  PINA = ptr_A;
  PINB = ptr_B;
  mtrSp = 0;
}

void DRVMOTOR::init(){
  pinMode(PINA, OUTPUT);
  pinMode(PINB, OUTPUT);
}

void DRVMOTOR::drive(int mtrSp){
  if(mtrSp > 0) {
    analogWrite(PINA, mtrSp);
    analogWrite(PINB, 0);
  }else if(mtrSp < 0) {
    analogWrite(PINA, 0);
    analogWrite(PINB, abs(mtrSp));
  }else{
    analogWrite(PINA, 0);
    analogWrite(PINB, 0);
  }
}