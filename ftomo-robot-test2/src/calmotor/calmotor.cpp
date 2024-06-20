#include "./calmotor.h"

// CALMOTOR::CALMOTOR(DRVMOTOR ptr_motor[]){
// 	// Constructor
// 	MTRS[0] = &ptr_motor[0];
// 	MTRS[1] = &ptr_motor[1];
// 	MTRS[2] = &ptr_motor[2];
// 	MTRS[3] = &ptr_motor[3];
// }

CALMOTOR::CALMOTOR(DRVMOTOR* ptr_motor){
	// Constructor
	MTRS[0] = ptr_motor;
	MTRS[1] = ptr_motor+1;
	MTRS[2] = ptr_motor+2;
	MTRS[3] = ptr_motor+3;
}

void CALMOTOR::init() {
	// Initialize the motor
	for(int i=0; i<4; i++){
		MTRS[i]->init();
	}
}

void CALMOTOR::setGo(int speed, int degree){
	// Set the motor to go
    degree = (degree + 180) % 360 - 180;
	for(int i=0; i<4; i++){
		MTRS[i]->drive(sin((degree + (135.0 - i * 90.0)) / 180.0 * 3.14) * speed);
	}
}