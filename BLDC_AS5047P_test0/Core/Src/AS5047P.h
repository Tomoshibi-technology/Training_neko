/*
 * AS5047P.h
 *
 *  Created on: Sep 16, 2025
 *      Author: RyukiTsuji
 */

#ifndef SRC_AS5047P_H_
#define SRC_AS5047P_H_

#include "main.h"

class AS5047P{
	public:
		AS5047P(SPI_HandleTypeDef*, GPIO_TypeDef*, uint16_t);
		void setup();//PWM出力して角度固定した状態でイニシャライズする必要
		uint16_t SPI_Read();
		void calc_angle();

		uint16_t get_initial_angle();
		uint16_t get_angle();

	private:
		SPI_HandleTypeDef* p_hspix;
		GPIO_TypeDef* GPIOx;
		uint16_t GPIO_Pin;

		uint16_t number_of_samples = 500;

		uint16_t initial_angle = 0;
		uint16_t angle = 0;
};

#endif /* SRC_AS5047P_H_ */
