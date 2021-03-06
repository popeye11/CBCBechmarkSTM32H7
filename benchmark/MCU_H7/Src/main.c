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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"
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
#define CTRLFREQUENCY 100e3
#define DAC_ALIGNMENT DAC_ALIGN_12B_R
#define hdac hdac1
#define TIM2PSC 0
uint32_t adc_value=0;
uint32_t dacvalue=0;
double volt_value=0;
uint32_t Tim2ARR;
uint32_t ADCBuf[2];
uint32_t   acValBuffer;
uint16_t DACoutput1;
uint16_t DACoutput2;
double paramFreq;
double   PT_Anf=0;
double   PT_End =0;
double   duration =0;
tPID* 		 pPID1       =   &InstancePID1;
tPID* 		 pPID2       =   &InstancePID2;
uint8_t PIDInputOption = 1;
double LockInOutput = 0.0;
double yout;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
		PT_Anf =PT_End;
		adc_value =(float)ADCBuf[0];
		volt_value=3.3f*((float)(adc_value)/65536); // single ended mode
		PID_Calc(pPID1,PIDInputOption, volt_value,LockInOutput);
		DACoutput1 = (uint16_t)(InstancePID1.outvalue*VOLT2DIG12BIT);
		DACoutput2 = (uint16_t)(InstancePID1.outvalue*VOLT2DIG12BIT);

		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_0);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGNMENT, (uint32_t)DACoutput1);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGNMENT, (uint32_t)DACoutput1);
		PT_End =DWT->CYCCNT;
		duration = PT_End-PT_Anf;
        //HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{


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
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_ADC3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 1;
  Tim2ARR=200e6 / CTRLFREQUENCY-1;
  __HAL_TIM_SET_PRESCALER(&htim2, TIM2PSC);
  __HAL_TIM_SET_AUTORELOAD(&htim2,  Tim2ARR);
  paramFreq = CTRLFREQUENCY;
  PID_vInit(pPID1);
  /* USER CODE BEGIN 1 */
	 // HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim2);
  //HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	      /* Start DAC */
  HAL_ADC_Start_DMA(&hadc1,ADCBuf,2);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
  HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
  //HAL_UART_Receive_DMA(&huart3,aRxBuffer,RXBUFFERSIZE);

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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.PLL3.PLL3M = 2;
  PeriphClkInitStruct.PLL3.PLL3N = 54;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 2;
  PeriphClkInitStruct.PLL3.PLL3R = 6;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
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
	pPID-> _Kp				= 		0.2;
	pPID-> _Ki				= 		0;
	pPID-> _Kd				= 		0.5;
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
