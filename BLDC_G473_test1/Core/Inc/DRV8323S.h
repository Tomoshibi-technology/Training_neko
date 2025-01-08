/*
 * DRV8323S.h
 *
 *  Created on: Jan 7, 2025
 *      Author: RyukiTsuji
 */

#ifndef SRC_DRV8323S_H_
#define SRC_DRV8323S_H_

#include "main.h"
//#include "stm32g4xx_hal_conf.h"
//#include "stm32g4xx_it.h"
//#include "stm32g4xx_hal.h"
#include <math.h>

class DRV8323S {
public:
	DRV8323S(SPI_HandleTypeDef* spi_handle);
private:
	SPI_HandleTypeDef* _hspi;

	uint8_t FaultStatus1_Add;
	uint8_t FaultStatus2_Add;
	uint8_t DriverCtrl_Add;
	uint8_t HSGateDrive_Add;
	uint8_t LSGateDrive_Add;
	uint8_t OCPCtrl_Add;
	uint8_t CSACtrl_Add;

	uint16_t DriverCtrl_Value;
	uint16_t HSGateDrive_Value;
	uint16_t LSGateDrive_Value;
	uint16_t OCPCtrl_Value;
	uint16_t CSACtrl_Value;


};

#endif /* SRC_DRV8323S_H_ */
