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
#include "AS5047P.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <stm32g4xx_hal_cordic.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define n_polepairs 7
#define FMAC_window 20

////PID gain
//#define K_p_omega_mech 10
//#define K_i_omega_mech 10//multiplied T_s/2, 2^16
//
//#define K_p_i_d 1
//#define K_i_i_d 1//multiplied T_s/2
//
//#define K_p_i_q 1
//#define K_i_i_q 1//multiplied T_s/2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CORDIC_HandleTypeDef hcordic;

FMAC_HandleTypeDef hfmac;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

//SYSTEM//
//GPIO handler array
__GPIO_Handle_TypeDef DRV8323S_GPIO_Handle_Array[7];

//PWM handler array
__PWM_Handle_TypeDef DRV8323S_PWM_Handle_Array[3];

//time
uint32_t cnt_ovf_TIM3 = 0;

uint64_t t_prev_loop_100ns = 0;
uint64_t t_curr_loop_100ns = 0;
int64_t T_loop_100ns = 0;

uint64_t t_prev_100ns = 0;
uint64_t t_start_100ns = 0;
uint64_t t_end_100ns = 0;

int64_t T_s_100ns = 0;
int64_t T_proc_100ns = 0;

//LED
uint64_t t_Ltika_prev_100ns = 0;

uint16_t cnt_Ltika_0 = 0;
uint8_t flag_LED0 = 0;
uint8_t flag_LED1 = 0;

//angle
uint16_t theta_init = 0;

uint16_t theta_mech_curr = 0;
uint16_t theta_elec_curr = 0;
uint16_t theta_mech_prev = 0;
//uint16_t theta_elec_prev = 0;
uint32_t theta_mech_curr_x256 = 0;
uint32_t theta_mech_prev_x256 = 0;

//speed
int16_t omega_mech_curr_x256 = 0;
//int16_t omega_elec_curr_x256 = 0;
int16_t omega_mech_prev_x256= 0;
//int16_t omega_elec_prev_x256 = 0;

//filtered speed
int16_t omega_mech_filt_curr_x256 = 0;
int16_t omega_mech_filt_prev_x256 = 0;

//FMAC
int16_t FMAC_coeff[FMAC_window] = {0};
int16_t FMAC_in = 0;
int16_t FMAC_out = 0;

//CORDIC
uint32_t CORDIC_in = 0;
uint32_t CORDIC_out = 0;
int16_t sincos_theta[2] = {0};//fixed point number, [0]=sin, [1]=cos
int32_t sincos_theta_xrt3[2] = {0};////multiplied root3, fixed point number, [0]=sin, [1]=cos

//ADC, current
uint16_t adc_raw_uvw[3] = {0};
int16_t i_uvw[3] = {0};
int16_t i_dq[2] = {0};

//PI
//PI gain
int32_t K_p_omega_mech = 100;
int32_t K_i_omega_mech = 30;//multiplied T_s/2, 2^10

int32_t K_p_i_d = 100;
int32_t K_i_i_d = 100;//multiplied T_s/2, 2^10

int32_t K_p_i_q = 100;
int32_t K_i_i_q = 100;//multiplied T_s/2, 2^10

int32_t anti_windup_omega_mech = 100000;
int32_t anti_windup_i_dq[2] = {50000, 50000};

//reference
int16_t omega_mech_ref_x256 = 0;
int16_t i_dq_ref[2] = {0};

//error
int32_t e_omega_mech_curr_x256 = 0;
int32_t e_omega_mech_prev_x256 = 0;
int32_t int_e_omega_mech_x256 = 0;

int32_t e_i_dq_curr[2] = {0};
int32_t e_i_dq_prev[2] = {0};
int32_t int_e_i_dq[2] = {0};

//voltage
int16_t v_uvw[3] = {0};
int16_t v_dq[2] = {0};

//output
uint16_t dutys[3] = {0};//0 ~ 3999 (limit ~3300)
//bool flag_bridge_hiZ[3] = {0};//0 or 1, if 0:Hi-Z/1:drive
bool flag_drive_slide = 0;
bool flag_drive_UI = 0;



//USER//
float RPS = 0.0;//Roll Per Second, if minus sign; reverse(-9 ~ 9)
float RPS_elec = 0.0;//Roll Per Second : electric, mul7


//DEBUG//
float onetime_variable = 0;

//int val_CCR1 = 0;
//int val_CCR2 = 0;
//int val_CCR3 = 0;
//
//int16_t val_past_ref_angle_mul10 = 0;//0 ~ 3599, past angle multiplied by 10
//int16_t val_ref_angle_mul10 = 0;//0 ~ 3599, now angle multiplied by 10
//int16_t val_ref_phases_mul10[3] = {0};//0 ~ 3599, reference each(UVW) phases
//
//int loop_counter = 0;//count loop times

//int reg_TIM1_DIER = 0x0;
//int reg_TIM1_SR = 0x0;
//int reg_TIM1_CCER = 0x0;
//int reg_TIM1_CR1 = 0x0;
//int reg_TIM1_CCMR2 = 0x0;
//
//int reg_TIM3_DIER = 0x0;
//int reg_TIM3_SR = 0x0;
//int reg_TIM3_CCER = 0x0;
//int reg_TIM3_CR1 = 0x0;
//int reg_TIM3_CCMR2 = 0x0;

//int reg_CORDIC_CSR = 0x0;

//int temp0 = 0;
//int temp1 = 0;

int duty_phaseA = 0;//~3300

//int temp_omega_mech_curr = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_CORDIC_Init(void);
static void MX_FMAC_Init(void);
/* USER CODE BEGIN PFP */
void DRV8323SHandleArray_setup(__GPIO_Handle_TypeDef*, __PWM_Handle_TypeDef*);
void FMAC_setup_FIR(int16_t*);
void write_FMAC(int16_t);
int16_t read_FMAC();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if (htim == &htim3){
    	cnt_ovf_TIM3 ++;
    }
}

//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
//    HAL_GPIO_TogglePin(TestPoint1_GPIO_Port, TestPoint1_Pin);
//}


void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
	//DEBUG//
	//calculate time
	t_start_100ns = (((uint64_t)cnt_ovf_TIM3) << 16) + (TIM3 -> CNT);
	T_s_100ns = t_start_100ns - t_prev_100ns;

	//toggle TP
	HAL_GPIO_TogglePin(TestPoint0_GPIO_Port, TestPoint0_Pin);
	//DEBUG//



	//get angle
	uint16_t Send_Data = 0xFFFFu;
	uint16_t Receive_Data = 0;
	HAL_GPIO_WritePin(CS1_Encoder_GPIO_Port, CS1_Encoder_Pin, GPIO_PIN_RESET);//CS1_MD->0
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)(&Send_Data), (uint8_t*)(&Receive_Data), 1, 100);//SPI send&receive
	HAL_GPIO_WritePin(CS1_Encoder_GPIO_Port, CS1_Encoder_Pin, GPIO_PIN_SET);//CS1_MD->1
	Receive_Data &= 0x3FFF; //mask angle data

	theta_mech_curr = (Receive_Data - theta_init) & 0x3FFFu; // scaling mechanical angle as 0 ~ 16383 (14bit)
	theta_elec_curr = ((theta_mech_curr * n_polepairs) & 0x3FFFu) << 2; // calculate electrical angle as 0 ~ 65535 (16bit)

	//start CORDIC
	CORDIC_in = theta_elec_curr; //substitute angle data for CORDIC argument (unsigned -> signed)
	CORDIC->WDATA = (0x7FFFu << 16) | CORDIC_in; //set ARG2 as m=1, and ARG1 (-1 ~ 1)

	//calc speed
	omega_mech_curr_x256 = ((int16_t)(theta_mech_curr - theta_mech_prev)) << 8;

	//start FMAC
	FMAC_in = omega_mech_curr_x256;
	write_FMAC(FMAC_in);

	//get current
	adc_raw_uvw[0] = ADC1->JDR1;
	adc_raw_uvw[1] = ADC1->JDR2;
	adc_raw_uvw[2] = ADC1->JDR3;

	i_uvw[0] = ((int16_t)(adc_raw_uvw[0]) - 2048) * 305 / 106;
	i_uvw[1] = ((int16_t)(adc_raw_uvw[1]) - 2048) * 305 / 106;
	i_uvw[2] = ((int16_t)(adc_raw_uvw[2]) - 2048) * 305 / 106;

	//read CORDIC
	while((CORDIC->CSR&0x80000000u)==0){};

	CORDIC_out = CORDIC->RDATA;
	sincos_theta[0] = (int16_t)(CORDIC_out);//RES1 = sin
	sincos_theta[1] = (int16_t)(CORDIC_out >> 16);//RES2 = cos

	//calc sincos root3 table : multiply by root3
	sincos_theta_xrt3[0] = sincos_theta[0] * 5042 / 2911;
	sincos_theta_xrt3[1] = sincos_theta[1] * 5042 / 2911;

	//uvw -> dq
	i_dq[0] = (( 2 * sincos_theta[1] * i_uvw[0]
			+ (-sincos_theta[1] + sincos_theta_xrt3[0]) * i_uvw[1]
			+ (-sincos_theta[1] - sincos_theta_xrt3[0]) * i_uvw[2]) / 3) >> 15;//[0] = i_d, gain = 40
	i_dq[1] = ((-2 * sincos_theta[0] * i_uvw[0]
			+ ( sincos_theta[0] + sincos_theta_xrt3[1]) * i_uvw[1]
			+ ( sincos_theta[0] - sincos_theta_xrt3[1]) * i_uvw[2]) / 3) >> 15;//[1] = i_q, gain = 40

	//read FMAC
	FMAC_out = read_FMAC();
	omega_mech_filt_curr_x256 = FMAC_out;
  
	//PI speed control : speed -> current
	e_omega_mech_curr_x256 = omega_mech_ref_x256 - omega_mech_filt_curr_x256;
	int_e_omega_mech_x256 += e_omega_mech_curr_x256;
	//i_d=0 control
	i_dq_ref[0] = 0;
	i_dq_ref[1] = (K_p_omega_mech * e_omega_mech_curr_x256
				+  K_i_omega_mech * int_e_omega_mech_x256) >> 10;

	//limit integer
	if(int_e_omega_mech_x256 > (anti_windup_omega_mech))
		{int_e_omega_mech_x256 = (anti_windup_omega_mech);}
	else if(int_e_omega_mech_x256 < -(anti_windup_omega_mech))
		{int_e_omega_mech_x256 = -(anti_windup_omega_mech);}
	else{int_e_omega_mech_x256 = int_e_omega_mech_x256;}

	//PI current control : current -> voltage
	//i_d control
	e_i_dq_curr[0] = i_dq_ref[0] - i_dq[0];
	int_e_i_dq[0] += e_i_dq_curr[0];

	v_dq[0] = (K_p_i_d * e_i_dq_curr[0]
			+  K_i_i_d * int_e_i_dq[0]) >> 10;

	//i_q control
	e_i_dq_curr[1] = i_dq_ref[1] - i_dq[1];
	int_e_i_dq[1] += e_i_dq_curr[1];

	v_dq[1] = (K_p_i_q * e_i_dq_curr[1]
			+  K_i_i_q * int_e_i_dq[1]) >> 10;

	//limit integer
	for(int i=0; i<2; i++){
		if(int_e_i_dq[i] > (anti_windup_i_dq[i]))
			{int_e_i_dq[i] = (anti_windup_i_dq[i]);}
		else if(int_e_i_dq[i] < -(anti_windup_i_dq[i]))
			{int_e_i_dq[i] = -(anti_windup_i_dq[i]);}
		else{int_e_i_dq[i] = int_e_i_dq[i];}
	}


	//dq -> uvw
	v_uvw[0] = (((2 * sincos_theta[1]) * v_dq[0]
			 + ( -2 * sincos_theta[0]) * v_dq[1]) >> 1) >> 15;//[0] = v_u
	v_uvw[1] = (((-sincos_theta[1] + sincos_theta_xrt3[0]) * v_dq[0]
			 + (   sincos_theta[0] + sincos_theta_xrt3[1]) * v_dq[1]) >> 1) >> 15;//[1] = v_v
	v_uvw[2] = (((-sincos_theta[1] - sincos_theta_xrt3[0]) * v_dq[0]
			 + (   sincos_theta[0] - sincos_theta_xrt3[1]) * v_dq[1]) >> 1) >> 15;//[2] = v_w

	//calc dutys
	int16_t __dutys[3] = {0};
	__dutys[0] = 1999 + v_uvw[0];
	__dutys[1] = 1999 + v_uvw[1];
	__dutys[2] = 1999 + v_uvw[2];

	//limit dutys
	for(int i=0; i<3; i++){
		if(__dutys[i] > 3300){dutys[i] = 3300;}
		else if(__dutys[i] < 0){dutys[i] = 0;}
		else{dutys[i] = __dutys[i];}
	}

	//read drive flag
	flag_drive_slide = HAL_GPIO_ReadPin(Slide0_GPIO_Port, Slide0_Pin);

	//set compare
	if((flag_drive_slide == 1) && (flag_drive_UI == 1)){
		HAL_GPIO_WritePin(INLA_GPIO_Port, INLA_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutys[0]);

		HAL_GPIO_WritePin(INLB_GPIO_Port, INLB_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, dutys[1]);

		HAL_GPIO_WritePin(INLC_GPIO_Port, INLC_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, dutys[2]);
	}else{
		HAL_GPIO_WritePin(INLA_GPIO_Port, INLA_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

		HAL_GPIO_WritePin(INLB_GPIO_Port, INLB_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

		HAL_GPIO_WritePin(INLC_GPIO_Port, INLC_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

		//reset integer
		int_e_omega_mech_x256 = 0;
		int_e_i_dq[0] = 0;
		int_e_i_dq[1] = 0;
	}


	//substitute NOW data for PAST data
	theta_mech_prev = theta_mech_curr;
	theta_mech_prev_x256 = theta_mech_curr_x256;
	omega_mech_prev_x256 = omega_mech_curr_x256;
	e_omega_mech_prev_x256 = e_omega_mech_curr_x256;



	//DEBUG//
    //calc time
	t_prev_100ns = t_start_100ns;
	t_end_100ns = (((uint64_t)cnt_ovf_TIM3) << 16) + (TIM3 -> CNT);
	T_proc_100ns = t_end_100ns - t_start_100ns;
	//DEBUG//
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
  MX_ADC1_Init();
  MX_CORDIC_Init();
  MX_FMAC_Init();
  /* USER CODE BEGIN 2 */

  //calc FMAC coeff
  for(int i=0; i<FMAC_window; i++){
      FMAC_coeff[i] = ((6 * (i+1) * (FMAC_window - i)) << 15)
                      /(FMAC_window * FMAC_window * FMAC_window
                    		  + 3 * FMAC_window * FMAC_window
							  + 2 * FMAC_window);
  }

  //set FMAC as FIR
  FMAC_setup_FIR(FMAC_coeff);

  //set ADC
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADC_Start(&hadc1);
//  HAL_ADCEx_InjectedStart_IT(&hadc1);

  //set timer
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
//  HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
//  TIM1 -> DIER |= 0b10000;
  TIM1 -> CCR4 = 3999 - 550; //set ADC trigger compare



  //set time counter
  t_curr_loop_100ns = (((uint64_t)cnt_ovf_TIM3) << 16) + (TIM3 -> CNT);
  t_prev_loop_100ns = t_curr_loop_100ns;
  t_Ltika_prev_100ns = t_curr_loop_100ns;

  //set LED
  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
  flag_LED0 = 1;
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  flag_LED1 = 0;



  //set Class
  DRV8323SHandleArray_setup(DRV8323S_GPIO_Handle_Array, DRV8323S_PWM_Handle_Array);//setting Handle Arrays
  DRV8323S drv8323s(DRV8323S_GPIO_Handle_Array, DRV8323S_PWM_Handle_Array, &hspi1);
  AS5047P as5047p(&hspi1, CS1_Encoder_GPIO_Port, CS1_Encoder_Pin);

  //standby
  HAL_Delay(2000);


  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //set DRV8323
  if(drv8323s.setup() == 0){
  }else{//ERROR
	  while(1){
		  HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		  flag_LED0 += 1;
		  flag_LED0 %= 2;

		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  flag_LED1 += 1;
		  flag_LED1 %= 2;

		  HAL_Delay(100);
	  }
  }

  //set AS5047P
  //lock motor
  drv8323s.PWM_OUT(0, GPIO_PIN_SET, 0);
  drv8323s.PWM_OUT(1, GPIO_PIN_SET, 0);
  drv8323s.PWM_OUT(2, GPIO_PIN_SET, 0);

  //slow start
  int lock_duty = 0;
  while(lock_duty < 600){
	  drv8323s.PWM_OUT(0, GPIO_PIN_SET, lock_duty);
	  drv8323s.PWM_OUT(1, GPIO_PIN_SET, 0);
	  drv8323s.PWM_OUT(2, GPIO_PIN_SET, 0);

	  lock_duty += 3;
	  HAL_Delay(1);
  }

  drv8323s.PWM_OUT(0, GPIO_PIN_SET, 600);
  drv8323s.PWM_OUT(1, GPIO_PIN_SET, 0);
  drv8323s.PWM_OUT(2, GPIO_PIN_SET, 0);
  HAL_Delay(2000);

  //set initial angle
  as5047p.setup();
  theta_init = as5047p.get_initial_angle();

  //unlock motor
  drv8323s.PWM_OUT(0, GPIO_PIN_RESET, 0);
  drv8323s.PWM_OUT(1, GPIO_PIN_RESET, 0);
  drv8323s.PWM_OUT(2, GPIO_PIN_RESET, 0);

  //standby
  HAL_Delay(800);

  //control start
  HAL_ADCEx_InjectedStart_IT(&hadc1);


  while (1)
  {
	// calculate 1　loop time//
	t_curr_loop_100ns = ((uint64_t)cnt_ovf_TIM3 << 16) + (TIM3 -> CNT);
	T_loop_100ns = t_curr_loop_100ns - t_prev_loop_100ns;
	t_prev_loop_100ns = t_curr_loop_100ns;

//	reg_TIM1_DIER = TIM1 -> DIER;
//	reg_TIM1_SR = TIM1 -> SR;
//	reg_TIM1_CCER = TIM1 -> CCER;
//	reg_TIM1_CR1 = TIM1 -> CR1;
//	reg_TIM1_CCMR2 = TIM1 -> CCMR2;
//
//	reg_TIM3_DIER = TIM3 -> DIER;
//	reg_TIM3_SR = TIM3 -> SR;
//	reg_TIM3_CCER = TIM3 -> CCER;
//	reg_TIM3_CR1 = TIM3 -> CR1;
//	reg_TIM3_CCMR2 = TIM3 -> CCMR2;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	drv8323s.PWM_OUT(0, GPIO_PIN_SET, duty_phaseA);
//	drv8323s.PWM_OUT(1, GPIO_PIN_SET, 0);
//	drv8323s.PWM_OUT(2, GPIO_PIN_SET, 0);

//	//output each Duty
//	for(uint8_t i=0; i<3; i++){
//		if(flag_bridge_HiZ[i] == 1){
//			drv8323s.PWM_OUT(i, GPIO_PIN_SET, dutys[i]);
//		}else{
//			drv8323s.PWM_OUT(i, GPIO_PIN_RESET, dutys[i]);
//		}
//	}

	//Lチカ//
	int temp0 = t_curr_loop_100ns - t_Ltika_prev_100ns;
	if(temp0 > 10000000){
		HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
		flag_LED0 += 1;
		flag_LED0 %= 2;

		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		flag_LED1 += 1;
		flag_LED1 %= 2;

		cnt_Ltika_0 ++;
		t_Ltika_prev_100ns = t_curr_loop_100ns;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 3;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO2;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CORDIC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CORDIC_Init(void)
{

  /* USER CODE BEGIN CORDIC_Init 0 */

  /* USER CODE END CORDIC_Init 0 */

  /* USER CODE BEGIN CORDIC_Init 1 */

  /* USER CODE END CORDIC_Init 1 */
  hcordic.Instance = CORDIC;
  if (HAL_CORDIC_Init(&hcordic) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CORDIC_Init 2 */
  CORDIC_ConfigTypeDef sCordicConfig;
  sCordicConfig.Function	= CORDIC_FUNCTION_SINE;    /* sinを計算 */
  sCordicConfig.Precision	= CORDIC_PRECISION_4CYCLES; 	/* 反復回数（計算精度）を設定する */
  sCordicConfig.Scale      	= CORDIC_SCALE_0;           /* scale設定。sinでは使用しない */
  sCordicConfig.NbWrite  	= CORDIC_NBWRITE_1;         	/* 入力データの数。1ならば位相のみ、2ならば位相θと径m[0,1]を設定できる */
  sCordicConfig.NbRead   	= CORDIC_NBREAD_1;          	/* 出力データの数。1がm*sinθ, 2がm*cosθ */
  sCordicConfig.InSize      = CORDIC_INSIZE_16BITS;     	/* 入力データの固定小数点位置 */
  sCordicConfig.OutSize   	= CORDIC_OUTSIZE_16BITS;    /* 出力データの固定小数点位置 */
  HAL_CORDIC_Configure(&hcordic, &sCordicConfig);
  /* USER CODE END CORDIC_Init 2 */

}

/**
  * @brief FMAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_FMAC_Init(void)
{

  /* USER CODE BEGIN FMAC_Init 0 */

  /* USER CODE END FMAC_Init 0 */

  /* USER CODE BEGIN FMAC_Init 1 */

  /* USER CODE END FMAC_Init 1 */
  hfmac.Instance = FMAC;
  if (HAL_FMAC_Init(&hfmac) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FMAC_Init 2 */

  /* USER CODE END FMAC_Init 2 */

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
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
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
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  HAL_GPIO_WritePin(GPIOB, MD_CAL_Pin|TestPoint0_Pin|TestPoint1_Pin|LED0_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : CS1_Encoder_Pin CS1_MD_Pin MD_CAL_Pin TestPoint0_Pin
                           TestPoint1_Pin LED0_Pin */
  GPIO_InitStruct.Pin = CS1_Encoder_Pin|CS1_MD_Pin|MD_CAL_Pin|TestPoint0_Pin
                          |TestPoint1_Pin|LED0_Pin;
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

void FMAC_setup_FIR(int16_t* coeff){
  uint32_t reg;

  /* reset FMAC */
  reg = FMAC->CR;
  reg &= ~(0x1 << 16);// mask RESET bit
  reg |= (0x1 << 16);// RESET=1
  FMAC->CR = reg;

  HAL_Delay(1);// wait for reset

  reg = FMAC->CR;
  reg &= ~(0x1 << 16);// mask RESET bit
  reg |= (0x0 << 16);// RESET=0
  FMAC->CR = reg;


  /* set FMAC config */
  //reset START bit
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x0 << 31);// START=0
  FMAC->PARAM = reg;

  //set CR register
  reg = FMAC->CR;
  reg &= ~(0x0000831F);// mask CLIPEN, DMA, IRQ bits
  reg |= (0x1 << 15);// CLIPEN=1
  FMAC->CR = reg;

  //set X1BUFCFG register
  reg = FMAC->X1BUFCFG;
  reg &= ~((0x3 << 24) | (0xFF << 8) | 0xFF);// mask FULL_WM, BUF_SIZE, BASE bits
  reg |= ((0x0 << 24) | (FMAC_window << 8) | FMAC_window);// FULL_WM=0, SIZE=20, BASE=20
  FMAC->X1BUFCFG = reg;

  //set X2BUFCFG register
  reg = FMAC->X2BUFCFG;
  reg &= ~((0xFF << 8) | 0xFF);//mask BUF_SIZE, BASE bits
  reg |= ((FMAC_window << 8) | 0x0);// SIZE=20, BASE=0
  FMAC->X2BUFCFG = reg;

  //set YBUFCFG register
  reg = FMAC->YBUFCFG;
  reg &= ~((0x3 << 24) | (0xFF << 8) | 0xFF);// mask FULL_WM, BUF_SIZE, BASE bits
  reg |= ((0x0 << 24) | (0x1 << 8) | FMAC_window*2);// FULL_WM=0, SIZE=1, BASE=40
  FMAC->YBUFCFG = reg;


  /* start preload */
  //preload X1 with 0
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x0 << 31);// START=0
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~((0x7F << 24) | 0xFF);// mask FUNC, P bits
  reg |= ((0x1 << 24) | FMAC_window);// FUNC=1(X1), P=20
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x1 << 31);// START=1
  FMAC->PARAM = reg;

  for (int i=0; i<FMAC_window; i++) {
      WRITE_REG(FMAC->WDATA, 0);
  }
  while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}

  // preload X2 with coeff[]
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x0 << 31);// START=0
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~((0x7F << 24) | (0xFF << 8) | 0xFF);// mask FUNC, Q, P bits
  reg |= ((0x2 << 24) | (0x0 << 8) | FMAC_window);// FUNC=2(X2), Q=0, P=20
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x1 << 31);// START=1
  FMAC->PARAM = reg;

  for (int i=0; i<FMAC_window; i++) {
      WRITE_REG(FMAC->WDATA, coeff[i]);
  }
  while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}

  // preload Y with 0
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x0 << 31);// START=0
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~((0x7F << 24) | 0xFF);// mask FUNC, P bits
  reg |= ((0x3 << 24) | 0x1);// FUNC=3(Y), P=1
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x1 << 31);// START=1
  FMAC->PARAM = reg;

  WRITE_REG(FMAC->WDATA, 0);
  while ((READ_REG(FMAC->PARAM) & (0x1 << 31)) != 0) {}


  /* set FIR */
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x0 << 31);// START=0
  FMAC->PARAM = reg;

  reg = FMAC->PARAM;
  reg &= ~((0x7F << 24) | (0xFF << 16) | 0xFF);// mask FUNC, R, P bits
  reg |= ((0x8 << 24) | (0x0 << 16) | FMAC_window);// FUNC=8(FIR), R=0, P=20
  FMAC->PARAM = reg;

  //start FIR
  reg = FMAC->PARAM;
  reg &= ~(0x1 << 31);// mask START bit
  reg |= (0x1 << 31);// START=1
  FMAC->PARAM = reg;

  //read once to start processing
  int32_t temp = READ_REG(FMAC->RDATA);

  //wait for FIR to complete
  while ((READ_REG(FMAC->SR) & 0x1) != 0) {}

  //read result to clear Y buffer
  temp = READ_REG(FMAC->RDATA);
}

void write_FMAC(int16_t data_in){
  int32_t temp = 0;

  if((FMAC->SR & (0x1)) == 0){ // YEMPTY==0
    if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0 : Yバッファが満杯、Xバッファは空
      // RDATAを1回読み出す
      temp = READ_REG(FMAC->RDATA);

    }else{ // X1FULL!=0 : Yバッファが満杯、Xバッファも満杯
      // RDATAを1回読み出す
      temp = READ_REG(FMAC->RDATA);
      // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
      while((FMAC->SR & (0x1)) != 0){}
      // RDATAをもう1回読み出す
      temp = READ_REG(FMAC->RDATA);
    }

  }else{ // YEMPTY!=0
    if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0 : Yバッファが空、Xバッファも空
      // そのまま書き込める

    }else{ // X1FULL!=0 : Yバッファが空、Xバッファは満杯
      // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
      while((FMAC->SR & (0x1)) != 0){}
      // RDATAを1回読み出す
      temp = READ_REG(FMAC->RDATA);
      // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
      while((FMAC->SR & (0x1)) != 0){}
      // RDATAをもう1回読み出す
      temp = READ_REG(FMAC->RDATA);
      while((FMAC->SR & (1 << 1)) != 0){}// X1FULLがクリアされるまで待つ
    }
  }
  // WDATAに最新データを書き込む
  FMAC->WDATA = WRITE_REG(FMAC->WDATA, data_in);
}

int16_t read_FMAC(){
  int32_t temp = 0;
  int32_t data_out = 0;

  while((FMAC->SR & (0x1)) != 0){} // YEMPTY=0になるまで待つ
  if((FMAC->SR & (0x1 << 1)) == 0){ // X1FULL==0
    // 正常に最新データが処理されていると判断できる
    data_out = READ_REG(FMAC->RDATA);
  } else { // X1FULL!=0
    // 最新データが処理されていない可能性がある
    // RDATAを1回読み出して捨てる
    temp = READ_REG(FMAC->RDATA);
    // FMACが停止してYEMPTYが0になる(結果が出力されてYバッファが埋まる)まで待つ
    while((FMAC->SR & (0x1)) != 0){}
    // RDATAをもう1回読み出す
    data_out = READ_REG(FMAC->RDATA);
  }
  return (int16_t)data_out;
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
