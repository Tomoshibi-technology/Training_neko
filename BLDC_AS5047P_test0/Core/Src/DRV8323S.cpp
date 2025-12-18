/*
 * DRV8323S.cpp
 *
 *  Created on: Jan 7, 2025
 *      Author: RyukiTsuji
 */

#include "DRV8323S.h"

DRV8323S::DRV8323S(__GPIO_Handle_TypeDef* ptr_GPIO_Handle_Array, __PWM_Handle_TypeDef* ptr_PWM_Handle_Array,
																				SPI_HandleTypeDef* ptr_hspix){
    GPIO_Handle_Array = ptr_GPIO_Handle_Array;
    PWM_Handle_Array = ptr_PWM_Handle_Array;
    p_hspix = ptr_hspix;
}


bool DRV8323S::setup(){//起動失敗->1, 起動完了->0
	//起動
	ENABLE(1);

	//nFAULTピンのチェック
	if(nFAULT() == 1){//正常に起動
	}else{//エラー
		return 1;
	}

	//障害検知レジスタのチェック(したい)

	//レジスタへの書き込み
	SPI_Write(0x02, Reg_0x02_init);
	SPI_Write(0x03, Reg_0x03_init);
	SPI_Write(0x04, Reg_0x04_init);
	SPI_Write(0x05, Reg_0x05_init);
	SPI_Write(0x06, Reg_0x06_init);
	HAL_Delay(10);

	//アンプのキャリブレーション
	CAL();

	return 0;
}


void DRV8323S::SPI_Write(uint8_t Reg_Add, uint16_t Reg_Data){
	//アドレスとデータの合成
	uint16_t Send_Data;
	Send_Data = 0 << 15 | Reg_Add << 11 | Reg_Data;

	//送信開始
	HAL_GPIO_WritePin(GPIO_Handle_Array[3].GPIOx, GPIO_Handle_Array[3].GPIO_Pin, GPIO_PIN_RESET);//CS1_MD->0
	HAL_SPI_Transmit(p_hspix, (uint8_t*)(&Send_Data), 1, 100);//SPI送信
	HAL_GPIO_WritePin(GPIO_Handle_Array[3].GPIOx, GPIO_Handle_Array[3].GPIO_Pin, GPIO_PIN_SET);//CS1_MD->1

	//待機
	HAL_Delay(1);
}


int DRV8323S::SPI_Read(){
	return 0;
}


void DRV8323S::ENABLE(bool enable_sig){//1->起動, 0->スリープ
	//起動信号
	if(enable_sig == 1){//ENABLE == High で起動
		HAL_GPIO_WritePin(GPIO_Handle_Array[0].GPIOx, GPIO_Handle_Array[0].GPIO_Pin, GPIO_PIN_SET);
	}else{//ENABLE == Low or else でスリープ
		HAL_GPIO_WritePin(GPIO_Handle_Array[0].GPIOx, GPIO_Handle_Array[0].GPIO_Pin, GPIO_PIN_RESET);
	}
//	if(enable_sig == 1){//ENABLE == High
//		uint32_t temp =  GPIO_Handle_Array[0]->GPIOx->ODR & GPIO_Handle_Array[0]->GPIO_Pin;
//		GPIO_Handle_Array[0]->GPIOx->BSRR = temp | GPIO_Handle_Array[0]->GPIO_Pin;//orを取る意味？
//	}else{//ENABLE == Low or else
//		uint32_t temp =  GPIO_Handle_Array[0]->GPIOx->ODR & GPIO_Handle_Array[0]->GPIO_Pin;
//		//GPIO_Handle_Array[0]->GPIOx->BRR = temp; //BRRは非推奨なので、BSRRを使う
//		GPIO_Handle_Array[0]->GPIOx->BSRR = temp << 16;
//	}

	//起動後待機
	HAL_Delay(5);
}


bool DRV8323S::nFAULT(){//FAULT検知->0, 正常動作->1
	bool nFAULT_flag = HAL_GPIO_ReadPin(GPIO_Handle_Array[1].GPIOx, GPIO_Handle_Array[1].GPIO_Pin);
	return nFAULT_flag;
}


void DRV8323S::CAL(){
	//出力オフ:INLx->0, INHx->0(PWMのDutyを0に)
	PWM_OUT(0, GPIO_PIN_RESET, 0);
	PWM_OUT(1, GPIO_PIN_RESET, 0);
	PWM_OUT(2, GPIO_PIN_RESET, 0);

	//一旦待ち
	HAL_Delay(1);

	//キャリブレーション開始
	HAL_GPIO_WritePin(GPIO_Handle_Array[2].GPIOx, GPIO_Handle_Array[2].GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);//待機

	//キャリブレーション終了
	HAL_GPIO_WritePin(GPIO_Handle_Array[2].GPIOx, GPIO_Handle_Array[2].GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}


void DRV8323S::PWM_OUT(uint8_t bridge, GPIO_PinState INL_sig, uint16_t compare){//bridge:0->A, 1->B, 2->C
													//INL_sig:GPIO_PIN_RESET(0)->Hi-Z, GPIO_PIN_SET(1)->Activate
	if(bridge < 3){//配列の範囲を指定
		//指定されたブリッジのINLxをINL_sigにセット
		HAL_GPIO_WritePin(GPIO_Handle_Array[4+bridge].GPIOx, GPIO_Handle_Array[4+bridge].GPIO_Pin, INL_sig);

		//指定されたブリッジのINHxのCCRをcompareにセット
		__DRV8323S_TIM_SET_COMPARE(PWM_Handle_Array[bridge].TIMx, PWM_Handle_Array[bridge].TIM_Channel, compare);
	}else{}
}


int DRV8323S::ADC_Read(){
	return 0;
}
