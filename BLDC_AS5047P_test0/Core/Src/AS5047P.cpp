/*
 * AS5047P.cpp
 *
 *  Created on: Sep 16, 2025
 *      Author: RyukiTsuji
 */

#include "AS5047P.h"

AS5047P::AS5047P(SPI_HandleTypeDef* ptr_hspix, GPIO_TypeDef* ptr_GPIOx, uint16_t gpiopin) {
    p_hspix = ptr_hspix;
	GPIOx = ptr_GPIOx;
	GPIO_Pin = gpiopin;
}

void AS5047P::setup(){//PWM出力して角度固定した状態でイニシャライズする必要
	uint16_t sampling[number_of_samples] = {};
	uint64_t total = 0;
	uint64_t mean = 0;

	for(int i=0; i<number_of_samples; i++){//サンプリングして合計
		sampling[i] = SPI_Read();
		total += sampling[i];
	}
	mean = total / (uint64_t)(number_of_samples);//平均を算出
	initial_angle = mean;//初期値として設定
}

uint16_t AS5047P::SPI_Read(){
	uint16_t Send_Data = 0xFFFF;
	uint16_t Receive_Data = 0;

	//送信開始
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);//CS1_MD->0
	HAL_SPI_TransmitReceive(p_hspix, (uint8_t*)(&Send_Data), (uint8_t*)(&Receive_Data), 1, 100);//SPI送信・受信
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);//CS1_MD->1
	Receive_Data &= 0x3FFF;//受信したデータをマスクして角度を取り出す

	return Receive_Data;
}

void AS5047P::calc_angle(){
	angle = SPI_Read() - initial_angle;
}

uint16_t AS5047P::get_initial_angle(){
	return initial_angle;
}

uint16_t AS5047P::get_angle(){
	return angle;
}
