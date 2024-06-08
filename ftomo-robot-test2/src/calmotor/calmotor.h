#ifndef _H_CALMOTOR_
#define _H_CALMOTOR_

#include <Arduino.h>
#include "../drvmotor/drvmotor.h"

class CALMOTOR{
	private:
		DRVMOTOR *MTRS[4];
	public:
		CALMOTOR(DRVMOTOR ptr_motor[]);
		void init();
		void setGo(int speed, int degree);
};


#endif