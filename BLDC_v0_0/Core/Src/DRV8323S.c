/*
 * DRV8323S.c
 *
 *  Created on: Dec 21, 2025
 *      Author: RyukiTsuji
 */


#include "DRV8323S.h"

void drv8323s_Handler_setup(SPI_HandleTypeDef* ptr_hspix){
//DRV8323 SPI setup//
    drv8323s_p_hspix = ptr_hspix;

//DRV8323 GPIO setup//
	drv8323s_GPIO_Handle_Array[0].GPIOx = MD_ENABLE_GPIO_Port; //ENABLE
	drv8323s_GPIO_Handle_Array[0].GPIO_Pin = MD_ENABLE_Pin;

	drv8323s_GPIO_Handle_Array[1].GPIOx = MD_nFAULT_GPIO_Port; //nFAULT
	drv8323s_GPIO_Handle_Array[1].GPIO_Pin = MD_nFAULT_Pin;

	drv8323s_GPIO_Handle_Array[2].GPIOx = MD_CAL_GPIO_Port; //CAL
	drv8323s_GPIO_Handle_Array[2].GPIO_Pin = MD_CAL_Pin;

	drv8323s_GPIO_Handle_Array[3].GPIOx = CS1_MD_GPIO_Port; //CS1_MD
	drv8323s_GPIO_Handle_Array[3].GPIO_Pin = CS1_MD_Pin;

	drv8323s_GPIO_Handle_Array[4].GPIOx = INLA_GPIO_Port; //INLA
	drv8323s_GPIO_Handle_Array[4].GPIO_Pin = INLA_Pin;

	drv8323s_GPIO_Handle_Array[5].GPIOx = INLB_GPIO_Port; //INLB
	drv8323s_GPIO_Handle_Array[5].GPIO_Pin = INLB_Pin;

	drv8323s_GPIO_Handle_Array[6].GPIOx = INLC_GPIO_Port; //INLC
	drv8323s_GPIO_Handle_Array[6].GPIO_Pin = INLC_Pin;

//DRV8323 PWM setup, you need to change following manually//
	drv8323s_PWM_Handle_Array[0].TIMx = TIM1; //INHA
	drv8323s_PWM_Handle_Array[0].TIM_Channel = TIM_CHANNEL_1;//

	drv8323s_PWM_Handle_Array[1].TIMx = TIM1; //INHB
	drv8323s_PWM_Handle_Array[1].TIM_Channel = TIM_CHANNEL_2;

	drv8323s_PWM_Handle_Array[2].TIMx = TIM1; //INHC
	drv8323s_PWM_Handle_Array[2].TIM_Channel = TIM_CHANNEL_3;
}


bool drv8323s_setup(){//起動失敗->1, 起動完了->0
	//起動
	drv8323s_ENABLE(1);

	//nFAULTピンのチェック
	if(drv8323s_nFAULT() == 1){//正常に起動
	}else{//エラー
		return 1;
	}

	//障害検知レジスタのチェック(したい)

	//レジスタへの書き込み
	drv8323s_SPI_Write(0x02, drv8323s_Reg_0x02_init);
	drv8323s_SPI_Write(0x03, drv8323s_Reg_0x03_init);
	drv8323s_SPI_Write(0x04, drv8323s_Reg_0x04_init);
	drv8323s_SPI_Write(0x05, drv8323s_Reg_0x05_init);
	drv8323s_SPI_Write(0x06, drv8323s_Reg_0x06_init);
	HAL_Delay(10);

	//アンプのキャリブレーション
	drv8323s_CAL();
	return 0;
}


void drv8323s_SPI_Write(uint8_t Reg_Add, uint16_t Reg_Data){
	//アドレスとデータの合成
	uint16_t Send_Data;
	Send_Data = 0 << 15 | Reg_Add << 11 | Reg_Data;

	//送信開始
	HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[3].GPIOx, drv8323s_GPIO_Handle_Array[3].GPIO_Pin, GPIO_PIN_RESET);//CS1_MD->0
	HAL_SPI_Transmit(drv8323s_p_hspix, (uint8_t*)(&Send_Data), 1, 100);//SPI送信
	HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[3].GPIOx, drv8323s_GPIO_Handle_Array[3].GPIO_Pin, GPIO_PIN_SET);//CS1_MD->1

	//待機
	HAL_Delay(1);
}


int drv8323s_SPI_Read(){
	return 0;
}


void drv8323s_ENABLE(bool enable_sig){//1->起動, 0->スリープ
	//起動信号
	if(enable_sig == 1){//ENABLE == High で起動
		HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[0].GPIOx, drv8323s_GPIO_Handle_Array[0].GPIO_Pin, GPIO_PIN_SET);
	}else{//ENABLE == Low or else でスリープ
		HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[0].GPIOx, drv8323s_GPIO_Handle_Array[0].GPIO_Pin, GPIO_PIN_RESET);
	}
//	if(enable_sig == 1){//ENABLE == High
//		uint32_t temp =  drv8323s_GPIO_Handle_Array[0]->GPIOx->ODR & drv8323s_GPIO_Handle_Array[0]->GPIO_Pin;
//		drv8323s_GPIO_Handle_Array[0]->GPIOx->BSRR = temp | drv8323s_GPIO_Handle_Array[0]->GPIO_Pin;//orを取る意味？
//	}else{//ENABLE == Low or else
//		uint32_t temp =  drv8323s_GPIO_Handle_Array[0]->GPIOx->ODR & drv8323s_GPIO_Handle_Array[0]->GPIO_Pin;
//		//drv8323s_GPIO_Handle_Array[0]->GPIOx->BRR = temp; //BRRは非推奨なので、BSRRを使う
//		drv8323s_GPIO_Handle_Array[0]->GPIOx->BSRR = temp << 16;
//	}

	//起動後待機
	HAL_Delay(5);
}


bool drv8323s_nFAULT(){//FAULT検知->0, 正常動作->1
	bool nFAULT_flag = HAL_GPIO_ReadPin(drv8323s_GPIO_Handle_Array[1].GPIOx, drv8323s_GPIO_Handle_Array[1].GPIO_Pin);
	return nFAULT_flag;
}


void drv8323s_CAL(){
	//出力オフ:INLx->0, INHx->0(PWMのDutyを0に)
	PWM_OUT(0, GPIO_PIN_RESET, 0);
	PWM_OUT(1, GPIO_PIN_RESET, 0);
	PWM_OUT(2, GPIO_PIN_RESET, 0);

	//一旦待ち
	HAL_Delay(1);

	//キャリブレーション開始
	HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[2].GPIOx, drv8323s_GPIO_Handle_Array[2].GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(1);//待機

	//キャリブレーション終了
	HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[2].GPIOx, drv8323s_GPIO_Handle_Array[2].GPIO_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}


void drv8323s_PWM_OUT(uint8_t bridge, GPIO_PinState INL_sig, uint16_t compare){//bridge:0->A, 1->B, 2->C
													//INL_sig:GPIO_PIN_RESET(0)->Hi-Z, GPIO_PIN_SET(1)->Activate
	if(bridge < 3){//配列の範囲を指定
		//指定されたブリッジのINLxをINL_sigにセット
		HAL_GPIO_WritePin(drv8323s_GPIO_Handle_Array[4+bridge].GPIOx, drv8323s_GPIO_Handle_Array[4+bridge].GPIO_Pin, INL_sig);

		//指定されたブリッジのINHxのCCRをcompareにセット
		__DRV8323S_TIM_SET_COMPARE(drv8323s_PWM_Handle_Array[bridge].TIMx, drv8323s_PWM_Handle_Array[bridge].TIM_Channel, compare);
	}else{}
}


int drv8323s_ADC_Read(){
	return 0;
}

