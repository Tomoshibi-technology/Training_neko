#ifndef _H_DRVMOTOR_
#define _H_DRVMOTOR_

#include <Arduino.h>

class DRVMOTOR{
  private:
    int PINA;
    int PINB;
    int mtrSp;
  public:
    DRVMOTOR(int ptr_A, int ptr_B);
    void init();
    void drive(int mtrSp);
};

#endif