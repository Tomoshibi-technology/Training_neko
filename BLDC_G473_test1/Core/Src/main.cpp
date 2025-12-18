/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DRV8323S.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
//time count
uint64_t past_time_100n = 0;
uint64_t now_time_100n = 0;
int64_t delta_time_100n = 0;
uint32_t CNTresister_overflow = 0;

//GPIO handler array
__GPIO_Handle_TypeDef DRV8323S_GPIO_Handle_Array[7];
//PWM handler array
__PWM_Handle_TypeDef DRV8323S_PWM_Handle_Array[3];


//LED
uint64_t Ltika_past_time_100n = 0;
uint16_t Ltika_count0 = 0;
uint8_t LED0_flag = 0;
uint8_t LED1_flag = 0;





//output value//
uint16_t dutys[3] = {0};//0 ~ 3999
bool HiZ_nFlag[3] = {0};//0 or 1, if 0:Hi-Z/1:drive

//angle & phase//
float past_ref_angle_mul10 = 0.0;//0 ~ 3599, past angle multiplied by 10
float ref_angle_mul10 = 0.0;//0 ~ 3599, now angle multiplied by 10
int16_t ref_phases_mul10[3] = {0};//0 ~ 3599, reference each(UVW) phases

//user variable//
float RPS = 0.0;//Roll Per Second, if minus sign; reverse(-9 ~ 9)
float RPS_electric = 0.0;//Roll Per Second : electric, mul7
float roll_voltage_rate = 0.0;//PWM voltage rate


//???//
float onetime_variable = 0;

////value for check variable//
//int val_CCR1 = 0;
//int val_CCR2 = 0;
//int val_CCR3 = 0;
//
//int16_t val_past_ref_angle_mul10 = 0;//0 ~ 3599, past angle multiplied by 10
//int16_t val_ref_angle_mul10 = 0;//0 ~ 3599, now angle multiplied by 10
//int16_t val_ref_phases_mul10[3] = {0};//0 ~ 3599, reference each(UVW) phases
//
//int loop_counter = 0;//count loop times




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void DRV8323SHandleArray_setup(__GPIO_Handle_TypeDef*, __PWM_Handle_TypeDef*);
void cal_120dgrSquareWave(int phase_mul10, float PWM_rate, uint8_t channel, uint16_t* ptr_dutys, bool* ptr_HiZ_nFlag);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim3){
    	CNTresister_overflow++;
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  //DRV8323 setting
  DRV8323SHandleArray_setup(DRV8323S_GPIO_Handle_Array, DRV8323S_PWM_Handle_Array);//setting Handle Arrays
  DRV8323S drv8323s(DRV8323S_GPIO_Handle_Array, DRV8323S_PWM_Handle_Array, &hspi1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //timer setting
  now_time_100n = (uint64_t)CNTresister_overflow * 65536 + (TIM3 -> CNT);
  past_time_100n = now_time_100n;

  //LED setting
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  LED0_flag = 1;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  LED1_flag = 0;



  HAL_Delay(5000);

  //DRV8323 setting
  if(drv8323s.setup() == 0){

  }else{//ERROR
	  while(1){
		  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		  LED0_flag += 1;
		  LED0_flag %= 2;

		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  LED1_flag += 1;
		  LED1_flag %= 2;

		  HAL_Delay(100);
	  }
  }




  while (1)
  {
	// calculate 1_loop time//
	now_time_100n = (uint64_t)CNTresister_overflow * 65536 + (TIM3 -> CNT);
	delta_time_100n = now_time_100n - past_time_100n;
	past_time_100n = now_time_100n;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */





	//calculate now angle
	RPS_electric = RPS * 7;
	onetime_variable = RPS_electric * 3600.0 * (float)delta_time_100n * 0.0000001;
	ref_angle_mul10 = past_ref_angle_mul10 + onetime_variable;
	while(ref_angle_mul10 < 0.0 || ref_angle_mul10 > 3600.0){//regulate to 0~3600
		if(ref_angle_mul10 < 0.0){
			ref_angle_mul10 += 3600.0;
		}else if(ref_angle_mul10 > 3600.0){
			ref_angle_mul10 -= 3600.0;
		}
	}

	//calculate each phase
	ref_phases_mul10[0] = (int)ref_angle_mul10;
	ref_phases_mul10[1] = ref_angle_mul10 + 1200;
		ref_phases_mul10[1] %= 3600;//regulate to 0~3600
	ref_phases_mul10[2] = ref_angle_mul10 + 2400;
		ref_phases_mul10[2] %= 3600;//regulate to 0~3600

	//calculate each phase Duty
	for(uint8_t i=0; i<3; i++){
		cal_120dgrSquareWave(ref_phases_mul10[i], roll_voltage_rate, i, dutys, HiZ_nFlag);
	}

	//output each Duty
	for(uint8_t i=0; i<3; i++){
		if(HiZ_nFlag[i] == 1){
			drv8323s.PWM_OUT(i, GPIO_PIN_SET, dutys[i]);
		}else{
			drv8323s.PWM_OUT(i, GPIO_PIN_RESET, dutys[i]);
		}
	}


	//substitute now to past
	past_ref_angle_mul10 = ref_angle_mul10;






	//Lチカ//
	if(now_time_100n - Ltika_past_time_100n > 10000000){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		LED0_flag += 1;
		LED0_flag %= 2;

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		LED1_flag += 1;
		LED1_flag %= 2;

		Ltika_count0 ++;
		Ltika_past_time_100n = now_time_100n;
	}else{}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 3999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 15;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, INLA_Pin|INLC_Pin|INLB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS1_Encoder_Pin|CS1_MD_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MD_CAL_Pin|LED0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|MD_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : INLA_Pin INLC_Pin INLB_Pin */
  GPIO_InitStruct.Pin = INLA_Pin|INLC_Pin|INLB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : MD_nFAULT_Pin */
  GPIO_InitStruct.Pin = MD_nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MD_nFAULT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS1_Encoder_Pin CS1_MD_Pin MD_CAL_Pin LED0_Pin */
  GPIO_InitStruct.Pin = CS1_Encoder_Pin|CS1_MD_Pin|MD_CAL_Pin|LED0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Slide0_Pin DIP1_Pin DIP3_Pin DIP2_Pin
                           DIP0_Pin */
  GPIO_InitStruct.Pin = Slide0_Pin|DIP1_Pin|DIP3_Pin|DIP2_Pin
                          |DIP0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin MD_ENABLE_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|MD_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void cal_120dgrSquareWave(int phase_mul10, float PWM_rate, uint8_t channel, uint16_t* ptr_dutys, bool* ptr_HiZ_nFlag){
	float max_duty = 3999.0;

	//120 degree Square Wave like Sin Wave
	if(0 <= phase_mul10 && phase_mul10 < 1200){//0 ~ 120, High
	ptr_dutys[channel] = (int)(max_duty * PWM_rate);
	ptr_HiZ_nFlag[channel] = 1;//Enable

	}else if(1200 <= phase_mul10 && phase_mul10 < 2400){//120 ~ 240, Hi-Z
	ptr_dutys[channel] = 0;
	ptr_HiZ_nFlag[channel] = 0;//disable

	}else if(2400 <= phase_mul10 && phase_mul10 < 3600){//240 ~ 359, Low
	ptr_dutys[channel] = 0;
	ptr_HiZ_nFlag[channel] = 1;//Enable

	}else{//exception
	ptr_dutys[channel] = 0;
	ptr_HiZ_nFlag[channel] = 0;//disable
	}
}

void DRV8323SHandleArray_setup(__GPIO_Handle_TypeDef* ptr_GPIO_Handle_Array, __PWM_Handle_TypeDef* ptr_PWM_Handle_Array){
//DRV8323 GPIO setup//
	ptr_GPIO_Handle_Array[0].GPIOx = MD_ENABLE_GPIO_Port; //ENABLE
	ptr_GPIO_Handle_Array[0].GPIO_Pin = MD_ENABLE_Pin;

	ptr_GPIO_Handle_Array[1].GPIOx = MD_nFAULT_GPIO_Port; //nFAULT
	ptr_GPIO_Handle_Array[1].GPIO_Pin = MD_nFAULT_Pin;

	ptr_GPIO_Handle_Array[2].GPIOx = MD_CAL_GPIO_Port; //CAL
	ptr_GPIO_Handle_Array[2].GPIO_Pin = MD_CAL_Pin;

	ptr_GPIO_Handle_Array[3].GPIOx = CS1_MD_GPIO_Port; //CS1_MD
	ptr_GPIO_Handle_Array[3].GPIO_Pin = CS1_MD_Pin;

	ptr_GPIO_Handle_Array[4].GPIOx = INLA_GPIO_Port; //INLA
	ptr_GPIO_Handle_Array[4].GPIO_Pin = INLA_Pin;

	ptr_GPIO_Handle_Array[5].GPIOx = INLB_GPIO_Port; //INLB
	ptr_GPIO_Handle_Array[5].GPIO_Pin = INLB_Pin;

	ptr_GPIO_Handle_Array[6].GPIOx = INLC_GPIO_Port; //INLC
	ptr_GPIO_Handle_Array[6].GPIO_Pin = INLC_Pin;

//DRV8323 PWM setup, you need to change following manually//
	ptr_PWM_Handle_Array[0].TIMx = TIM1; //INHA
	ptr_PWM_Handle_Array[0].TIM_Channel = TIM_CHANNEL_1;//

	ptr_PWM_Handle_Array[1].TIMx = TIM1; //INHB
	ptr_PWM_Handle_Array[1].TIM_Channel = TIM_CHANNEL_2;

	ptr_PWM_Handle_Array[2].TIMx = TIM1; //INHC
	ptr_PWM_Handle_Array[2].TIM_Channel = TIM_CHANNEL_3;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
