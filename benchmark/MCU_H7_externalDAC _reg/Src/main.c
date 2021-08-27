/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */
#define DAC_ALIGNMENT DAC_ALIGN_12B_R
#define hdac hdac1
#define TIM2PSC 0
#define INIVALUE 			0
#define ACCUMULATORWIDTH    16
#define LUTGRIDWIDTH        10
#define BUFFER_LEN    200
#define PI          		3.141592
#define defrootTerm_LEN 	1024
signed int LUT_T[defrootTerm_LEN];
float      LUT_d[defrootTerm_LEN];
uint32_t adc_value=0;
uint32_t dacvalue=0;
double volt_value=0;
uint32_t Tim2ARR;
uint32_t ADCBuf[2];
uint16_t  pData; //{0x7530};  //3952
uint32_t   acValBuffer;
uint16_t DACoutput1;
uint16_t DACoutput2;
uint16_t li;
double paramFreq;
uint32_t   PT_Anf=0;
uint32_t   PT_End =0;
uint32_t   ADC_Anf=0;
uint32_t   ADC_End =0;
uint32_t   DAC_Anf=0;
uint32_t   DAC_End =0;
uint32_t   PID_Anf=0;
uint32_t   PID_End =0;
uint32_t   SPI_Anf=0;
uint32_t   SPI_End =0;
double duration_sum;
double   duration =0;
double  SPI_duration = 0;
double  ADC_duration = 0;
double  PID_duration = 0;
tPID* 		 pPID1       =   &InstancePID1;
tPID* 		 pPID2       =   &InstancePID2;
tDDS*        pDDS        =   &InstanceDDS;
uint8_t PIDInputOption = 1;
double LockInOutput = 0.0;
double yout;
uint16_t charI[16];
uint16_t hexString;
uint16_t pdata;
uint16_t li;
uint16_t spi_data = 0x0;
__IO uint16_t *ptxdr_16bits = (uint16_t*)&SPI1->TXDR; // for 16-bit-write

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */
float PIDInputSWitch(uint8_t PIDInputOption, float ADCvalue,float LockIn)
{
	 float PIDInput;
	 if (PIDInputOption == 0)
			 {
	 	 	 	 PIDInput=LockIn;
			 }
	else
			{
				PIDInput =ADCvalue;
			}
	 return(PIDInput);
}

void PID_vInit(tPID* pPID)
{                                          ///< Zeiger auf Reglerstruktur
	pPID->_Ts 				= 		1/CTRLFREQUENCY;
	pPID-> _Kp				= 		1;
	pPID-> _Ki				= 		0;
	pPID-> _Kd				= 		0.0;
	pPID-> _max				= 		3;
	pPID-> _min				= 		-3;
	pPID-> _Kaw				= 		0.0;
	pPID-> _EnKc			= 		1;
	pPID-> _fc				= 		10;
    pPID-> _PIDHold			= 		0.0;
	pPID->error             =        0;
	pPID->error_1lag        =        0;
	pPID->error_2lag        =        0;
	pPID->error_AnWi        =        0;
	pPID->preout            =        0;
	pPID->_kt               =        -1;
	pPID->En                =        1.0;
	pPID->ref               =       0.0;
	pPID->a0                =        0.0;
	pPID->a1 				= 		0.0;
	pPID->a2      			= 		0.0;
	pPID->aw				=		0.0;
	pPID->omega 			=		0;
	pPID->a0 				= 		pPID-> _Kp+pPID-> _Kd/V2MUV/pPID->_Ts+pPID->_Ki*V2MV*pPID->_Ts;
	pPID->a1 				= 		-(pPID-> _Kp+pPID-> _Kd/V2MUV/pPID->_Ts*2.0);
	pPID->a2 				=		pPID->_Kd/V2MUV/pPID->_Ts;
	pPID->aw   				=		pPID->_Kaw*pPID->_Ts;
};
void DDS_vInit(float amp, float freq, float fclk,int accumulatorWidth, int LUTGridWidth)
{
	tDDS* 		 pDDS       =   &InstanceDDS;
	float        temp      	=   0;
	signed int 	 SinN      	=   LUTGridWidth;
	float 		 StepNo    	=   (float)(1<<SinN);
	float        StepEnc   	=   (float)(2.0f*PI-2.0f*PI/(StepNo-1.0))/(StepNo-1);
	float        phi;
	float        phi_lag;
	pDDS->enable            =   1;
	pDDS->amp 				= 	amp;
	pDDS->freq 				= 	freq;
	pDDS->phaseOffset 		= 	100;
	pDDS->fclk 				= 	fclk;
	pDDS->accumulatorWidth 	=	accumulatorWidth;
	pDDS->LUTGridWidth 		= 	LUTGridWidth;
	phi               		=   pDDS->phaseOffset/180*PI;
	phi_lag                 =   pDDS->phaseOffset_1ag/180*PI;
	pDDS->TWSumShift        = 	0;
	pDDS->TWSum             = 	0;
	pDDS->phaseOffset_1ag   = 	INIVALUE;
	pDDS->fclk_Shift_ACCWidth = (pDDS->fclk)/(1<<pDDS-> accumulatorWidth);
	pDDS->pi_Shift_AccWidth   = 10;//(float)(1<<pDDS-> accumulatorWidth)/PI/2.0;
	pDDS->TW                =   round((pDDS->freq)/pDDS->fclk_Shift_ACCWidth);
	pDDS->DetTW             =   round((phi-phi_lag)*(1<<pDDS-> accumulatorWidth)/PI/2.0f);
    pDDS->Out               = 	0;
    pDDS->ShiftOut          =	0;
	for(int li=0; li<=(1<<LUTGridWidth)-1; li++)
	{
		LUT_T[li]=li;
		LUT_d[li]=sinf(temp);
		temp = temp+StepEnc;
     }
};
float LUTinterp(float fLUTIndex, signed int* LUTX, float *LUTd)
   {
       signed int *IdxLUTX = LUTX;
       float *IdxLUTd = LUTd;
       float root_INTERPOLATED;
       float lfm;
       int intLUTIndex= (int)fLUTIndex;
       float InputT  = fLUTIndex;
       if ((InputT < *LUTX) )
           {
               return 0;
           }

       lfm = (LUTd[intLUTIndex+1]- LUTd[intLUTIndex])/ (LUTX[intLUTIndex+1]- LUTX[intLUTIndex]);
       root_INTERPOLATED = ( lfm * (InputT- LUTX[intLUTIndex])) + LUTd[intLUTIndex] ;
       if (InputT>=(float)defrootTerm_LEN-1)
              {
           	   lfm = (( LUTd[0] - LUTd[defrootTerm_LEN-1]));
           	   root_INTERPOLATED = (( lfm * (InputT -LUTX[defrootTerm_LEN-1] ))) +  LUTd[defrootTerm_LEN-1];
              }
       return (root_INTERPOLATED);//(root_INTERPOLATED);
}

void DDS_Calc()
{
	tDDS* 		pDDS       =   &InstanceDDS;
	int 		phase_out;
	int 		phase_outShift;
	float 		LUTIndexShift;
	float 		LUTIndex;
	float       Offset=1;

	pDDS->TWSum               	=      pDDS->TWSum+pDDS->TW;
	phase_out           		=      pDDS->TWSum;
	if(pDDS->phaseOffset!=pDDS->phaseOffset_1ag)
	    {
		pDDS->TWSumShift       =      pDDS->TWSumShift+pDDS->TW+pDDS->DetTW;
		pDDS->phaseOffset_1ag  =      pDDS->phaseOffset;

	    }
	    else
	    {
	    	pDDS->TWSumShift       =       pDDS->TWSumShift+pDDS->TW;
	    }
	    phase_outShift      	=       pDDS->TWSumShift;
	    if (phase_outShift >  (1<<pDDS-> accumulatorWidth)-1)
	    	{
	        	phase_outShift       =      phase_outShift-(1<<pDDS-> accumulatorWidth)+1;//>>pDDS->accumulatorWidth;
	        	pDDS->TWSumShift        =     phase_outShift;
	    	}
	    if (phase_out >  (1<<pDDS-> accumulatorWidth)-1)
	       {
	        	phase_out       	=   phase_out-(1<<pDDS-> accumulatorWidth)+1;//>>pDDS->accumulatorWidth;
	        	pDDS->TWSum       =   phase_out;
	       }
	     LUTIndexShift   =   (float)phase_outShift/(float)((1<<(pDDS->accumulatorWidth-pDDS->LUTGridWidth)));
	     LUTIndex        =   (float)phase_out/(float)((1<<(pDDS->accumulatorWidth-pDDS->LUTGridWidth)));
	 if (pDDS->enable==1)
	 {
	     pDDS->Out =LUTinterp(LUTIndex, LUT_T, LUT_d)*pDDS->amp;
	     pDDS->ShiftOut =LUTinterp(LUTIndexShift, LUT_T, LUT_d)+Offset;
	 }
	 else
	 {
		 pDDS->Out=0;
		 pDDS->ShiftOut=0;
	 }
};

void PID_Calc(tPID* pPID,uint8_t PIDInputOption, double ADCvalue,double LockIn)
{
	double 			output;
	double 			outputsat;
	double          pd;
	pd          				= 	PIDInputSWitch(PIDInputOption, ADCvalue, LockIn);
	pPID->error = (pPID->ref - pd)*pPID->_kt;
	if (pPID->En>=1)
	{
		output = (pPID->preout+pPID->a0*pPID->error + pPID->a1*pPID->error_1lag + pPID->a2*pPID->error_2lag+pPID->aw*pPID->error_AnWi)/(1+pPID->omega*pPID->_Ts);
	}
	else
	{
		output = 0;
		pPID->error             =        0;
		pPID->error_1lag        =        0;
		pPID->error_2lag        =        0;
		pPID->preout            =        0;
	}
//	if( output > pPID->_max*3)
//		{
//		   output = 3*pPID->_max;
//		}
//	if( output < pPID->_min*3 )
//			{
//			   output = 3*pPID->_min;
//			}
	pPID->preout = output;
	    // Restrict to max/min
	if( output > pPID->_max )
	{
	   outputsat = pPID->_max;
	}
	else if( output < pPID->_min )
	{
	   outputsat = pPID->_min;
	}
	else
	{
	   outputsat = output;
	}
	pPID->error_AnWi              =   outputsat-output;
	    // Save error to previous error
	pPID->error_2lag = pPID->error_1lag;
	pPID->error_1lag = pPID->error;
//   tty3 =Kd/pPID->_Ts;
	pPID->outvalue= outputsat;
}
void SPI1_start()
{
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	  SPI1->CR1 |= SPI_CR1_SPE_Msk;    // enable SPI
	  SPI1->CR1 |= SPI_CR1_CSTART_Msk; // master transfer start

}
void SPI1_transmit(uint16_t data)
{
    GPIOB->BSRR = GPIO_PIN_4 << 16; // Reset
    *ptxdr_16bits = data;
    while( !(SPI1->SR & SPI_SR_TXC_Msk));  // check if FiFo transmission complete
    GPIOB->BSRR = GPIO_PIN_4; // Set

}
uint32_t ADC_read(ADC_HandleTypeDef *hadc)
{
    /* Enable the ADC peripheral */
	uint32_t adc_value;
/*    ADC_STATE_CLR_SET(hadc->State,
                        HAL_ADC_STATE_READY | HAL_ADC_STATE_REG_EOC | HAL_ADC_STATE_REG_OVR | HAL_ADC_STATE_REG_EOSMP,HAL_ADC_STATE_REG_BUSY);
   //__HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC | ADC_FLAG_EOS | ADC_FLAG_OVR));
    ADC1->ISR |=ADC_ISR_EOC_Msk;
    ADC1->CR |=ADC_CR_ADSTART_Msk;*/
   // MODIFY_REG(ADC1->CR,ADC_CR_BITS_PROPERTY_RS,ADC_CR_ADSTART);
	 ADC1->ISR |=ADC_ISR_EOC_Msk;
	 ADC1->CR |=ADC_CR_ADSTART_Msk;
	 while ((ADC1->ISR  & ADC_ISR_EOC_Msk) == 0UL);
	 adc_value =ADC1->DR;
   // ADC_STATE_CLR_SET(hadc->State,HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,HAL_ADC_STATE_READY);
    return adc_value;
}
HAL_StatusTypeDef HAL_ADC_Stop1(ADC_HandleTypeDef *hadc)
{
  /* 1. Stop potential conversion on going, on ADC groups regular and injected */
  ADC_ConversionStop(hadc, ADC_REGULAR_INJECTED_GROUP);
  ADC_Disable(hadc);
  ADC_STATE_CLR_SET(hadc->State,HAL_ADC_STATE_REG_BUSY | HAL_ADC_STATE_INJ_BUSY,HAL_ADC_STATE_READY);
    }
HAL_StatusTypeDef HAL_ADC_PollForConversion1(ADC_HandleTypeDef *hadc, uint32_t Timeout)
{

// ADC1->ISR |=	HAL_ADC_STATE_REG_EOC;
//  SET_BIT(hadc->State, HAL_ADC_STATE_REG_EOC);
 // __HAL_ADC_CLEAR_FLAG(hadc, (ADC_FLAG_EOC));
	 while ((ADC1->ISR  & ADC_ISR_EOC_Msk) == 0UL);
}

void HAL_TIM_IRQHandler1(TIM_HandleTypeDef *htim)
{

  /* TIM Update event */
  if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) != RESET)
  {
    if(__HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE) !=RESET)
    {
      __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
      HAL_TIM_PeriodElapsedCallback(htim);
    }
  }
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

/*
	  adc_value =HAL_ADC_GetValue(&hadc1);
	  volt_value=3.3f*((float)(adc_value)/65536); // single ended mode
*/

//	  DACOut=(uint32_t)(2.0/(3.3/65536));

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_0);

		//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_SET);
		SPI_Anf =DWT->CYCCNT;
		SPI1_transmit(DACoutput1);
		PT_Anf =PT_End;
		SPI_End =DWT->CYCCNT;
		adc_value=ADC_read(&hadc1);
		ADC_End =DWT->CYCCNT;
		volt_value = DIG2WOLT*(float)(adc_value);
		PID_Calc(pPID1,PIDInputOption, volt_value,LockInOutput);
		DDS_Calc();
	//	DACoutput1 = (uint16_t)(InstancePID1.outvalue*VOLT2DIG16BIT);
		DACoutput1=(uint16_t)(InstanceDDS.ShiftOut*VOLT2DIG16BIT); //
		PID_End=DWT->CYCCNT;
		PID_duration =(PID_End-ADC_End)*0.0025;
		ADC_duration = (ADC_End-SPI_End)*0.0025;
		SPI_duration = (SPI_End-SPI_Anf)*0.0025;
		duration_sum =PID_duration+ADC_duration+SPI_duration;
		PT_End = DWT->CYCCNT;
		duration = (PT_End-PT_Anf)*0.0025;
		//HAL_GPIO_WritePin(GPIOG,GPIO_PIN_0,GPIO_PIN_RESET);
      //  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  Tim2ARR=200e6 / CTRLFREQUENCY-1;
  __HAL_TIM_SET_PRESCALER(&htim2, TIM2PSC);
  __HAL_TIM_SET_AUTORELOAD(&htim2,  Tim2ARR);
  paramFreq = CTRLFREQUENCY;
  PID_vInit(pPID1);
  DDS_vInit(0.05, 1000.0, paramFreq, ACCUMULATORWIDTH, LUTGRIDWIDTH);
  /* USER CODE BEGIN 1 */
	 // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	      /* Start DAC */
  //HAL_ADC_Start_DMA(&hadc1,ADCBuf,2);
 // HAL_ADC_Start(&hadc1);
  ADC_Enable(&hadc1);
  SPI1_start();
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  //HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  //HAL_UART_Receive_DMA(&huart3,aRxBuffer,RXBUFFERSIZE);
 // HAL_SPI_Transmit_DMA(&hspi1, pData,2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_SPI4|RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 25;
  PeriphClkInitStruct.PLL3.PLL3P = 1;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 3;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.Spi45ClockSelection = RCC_SPI45CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
