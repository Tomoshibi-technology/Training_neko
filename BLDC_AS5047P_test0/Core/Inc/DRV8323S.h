/*
 * DRV8323S.h
 *
 *  Created on: Jan 7, 2025
 *      Author: RyukiTsuji
 */

#ifndef SRC_DRV8323S_H_
#define SRC_DRV8323S_H_

#include "main.h"


typedef struct{
    GPIO_TypeDef* GPIOx;
    uint16_t GPIO_Pin;
} __GPIO_Handle_TypeDef;

typedef struct{
    TIM_TypeDef* TIMx;
    uint8_t TIM_Channel;
} __PWM_Handle_TypeDef;

#define __DRV8323S_TIM_SET_COMPARE(__TYPEDEF__, __CHANNEL__, __COMPARE__) \
  (((__CHANNEL__) == TIM_CHANNEL_1) ? ((__TYPEDEF__)->CCR1 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_2) ? ((__TYPEDEF__)->CCR2 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_3) ? ((__TYPEDEF__)->CCR3 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_4) ? ((__TYPEDEF__)->CCR4 = (__COMPARE__)) :\
   ((__CHANNEL__) == TIM_CHANNEL_5) ? ((__TYPEDEF__)->CCR5 = (__COMPARE__)) :\
   ((__TYPEDEF__)->CCR6 = (__COMPARE__)))


class DRV8323S{
    public:
        DRV8323S(__GPIO_Handle_TypeDef*, __PWM_Handle_TypeDef*, SPI_HandleTypeDef*);
        bool setup();//起動失敗->1, 起動完了->0
        void SPI_Write(uint8_t, uint16_t);
        int SPI_Read();
        void ENABLE(bool);//1->起動, 0->スリープ
        bool nFAULT();//FAULT検知->0, 正常動作->1
        void CAL();
        void PWM_OUT(uint8_t, GPIO_PinState, uint16_t);//bridge:0->A, 1->B, 2->C
														//INL_sig:GPIO_PIN_RESET(0)->Hi-Z, GPIO_PIN_SET(1)->Activate
        int ADC_Read();

    private:
        __GPIO_Handle_TypeDef* GPIO_Handle_Array;//0:ENABLE_Handle, 1:nFAULT_Handle, 2:CAL_Handle, 3:CS1_MD_Handle,
        																//4:INLA_Handle, 5:INLB_Handle, 6:INLC_Handle
        __PWM_Handle_TypeDef* PWM_Handle_Array;//0:INHA_Handle, 1:INHB_Handle, 2:INHC_Handle
        SPI_HandleTypeDef* p_hspix;

        uint16_t Reg_0x02_init = 0b00010100000;//Driver Control Register 0x02
        uint16_t Reg_0x03_init = 0b01111001100;//Gate HIGH Side Register 0x03
        uint16_t Reg_0x04_init = 0b10111001100;//Gate LOW Side Register 0x04
        uint16_t Reg_0x05_init = 0b01100001001;//OCP Register 0x05
        uint16_t Reg_0x06_init = 0b01011000000;//CSA Register 0x06
};


#endif /* SRC_DRV8323S_H_ */
