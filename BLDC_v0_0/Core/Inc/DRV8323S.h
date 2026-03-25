/*
 * DRV8323S.h
 *
 *  Created on: Dec 21, 2025
 *      Author: RyukiTsuji
 */

#ifndef INC_DRV8323S_H_
#define INC_DRV8323S_H_


#include "main.h"
#include <stdbool.h>

//構造体の宣言
typedef struct{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
} __GPIO_Handle_TypeDef;

typedef struct{
    TIM_TypeDef* TIMx;
    uint8_t TIM_Channel;
} __PWM_Handle_TypeDef;

//DRV8323S用のハンドラの定義
//GPIO handler array
__GPIO_Handle_TypeDef drv8323s_GPIO_Handle_Array[7];
//PWM handler array
__PWM_Handle_TypeDef drv8323s_PWM_Handle_Array[3];
//SPI handler
SPI_HandleTypeDef* drv8323s_p_hspix;

//レジスタ設定
uint16_t drv8323s_Reg_0x02_init = 0b00010100000;//Driver Control Register 0x02
uint16_t drv8323s_Reg_0x03_init = 0b01111001100;//Gate HIGH Side Register 0x03
uint16_t drv8323s_Reg_0x04_init = 0b10111001100;//Gate LOW Side Register 0x04
uint16_t drv8323s_Reg_0x05_init = 0b01100001001;//OCP Register 0x05
uint16_t drv8323s_Reg_0x06_init = 0b01011000000;//CSA Register 0x06

//関数プロトタイプ宣言
void drv8323s_Handler_setup(SPI_HandleTypeDef*);
bool drv8323s_setup();
void drv8323s_SPI_Write(uint8_t, uint16_t);
int drv8323s_SPI_Read();
void drv8323s_ENABLE(bool);//1->起動, 0->スリープ
bool drv8323s_nFAULT();//FAULT検知->0, 正常動作->1
void drv8323s_CAL();
void drv8323s_PWM_OUT(uint8_t, GPIO_PinState, uint16_t);//bridge:0->A, 1->B, 2->C
                                                //INL_sig:GPIO_PIN_RESET(0)->Hi-Z, GPIO_PIN_SET(1)->Activate
int drv8323s_ADC_Read();


#endif /* INC_DRV8323S_H_ */
