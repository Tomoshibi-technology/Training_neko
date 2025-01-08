/*
 * DRV8323S.cpp
 *
 *  Created on: Jan 7, 2025
 *      Author: RyukiTsuji
 */

#include "DRV8323S.h"
//#include "stm32g4xx_hal.h"
//#include "main.h"

DRV8323S::DRV8323S(SPI_HandleTypeDef* spi_handle)
	: _hspi(spi_handle){

	FaultStatus1_Add = 0x00;
	FaultStatus2_Add = 0x01;
	DriverCtrl_Add = 0x02;
	HSGateDrive_Add = 0x03;
	LSGateDrive_Add = 0x04;
	OCPCtrl_Add = 0x05;
	CSACtrl_Add = 0x06;

	DriverCtrl_Value = 0b00010100000;
	HSGateDrive_Value = 0b01111001100;
	LSGateDrive_Value = 0b10111001100;
	OCPCtrl_Value = 0b01100001001;
	CSACtrl_Value = 0b1011000000;
}
