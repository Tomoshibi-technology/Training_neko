/*
 * AS5047P.h
 *
 *  Created on: Dec 21, 2025
 *      Author: RyukiTsuji
 */

#ifndef INC_AS5047P_H_
#define INC_AS5047P_H_

void setup();//PWM出力して角度固定した状態でイニシャライズする必要
uint16_t SPI_Read();
void calc_angle();

uint16_t get_initial_angle();
uint16_t get_angle();

#endif /* INC_AS5047P_H_ */
