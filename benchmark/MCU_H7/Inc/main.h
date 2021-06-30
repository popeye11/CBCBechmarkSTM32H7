/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#define DAC_ALIGNMENT DAC_ALIGN_12B_R
#define hdac hdac1
#define TIM2PSC 0
#define CTRLFREQUENCY 2.0e5
//#define ADC2DACFREQ  5
#define TXBUFFERSIZE 40
#define RXBUFFERSIZE 1
#define PSBUFFERSIZE 40
#define OVS   960
#define PID_KP 65.745
#define PID_KI 28.0451           // discrete  KI = KI*Ts= 28.0451/(1e6/960)=0.0269
#define PID_KD 1.0869            // discrete  Kd =Kd/Ts= 1.0869*(1e6/960)
#define PID_KAW 0.02             // discrete anti windup coefficient
#define TS     OVS/1.0e6
#define MODULATIONAMP 3
#define DIG2VOLT16BIT 5.0355e-05  // 3.3[v]/65535 ->16bit
#define VOLT2DIG12BIT 1.2409e+03 //3.3[v]/4096->12bit
#define VOLT2DIG16BIT 1.9860e+04 //3.3[v]/2^16->16bit
#define DAC_CHANNEL DAC_CHANNEL_2
#define MAXCOUNT14BIT  4-1
#define MAXCOUNT16BIT  16-1
#define BITSHIFT14BIT  2
#define BITSHIFT16BIT  4
#define VOLTOFFSET     3.3/2
#define PIDOUTLIM      1.55    // voltage upper limt 3V-3.3V/2
#define PARAMSIZE      41
uint8_t Switch1_ModulationEnable;
uint8_t Switch1_offsetEnable;
uint8_t Switch2_offsetEnable;
uint8_t Switch1_PID1Enable;
uint8_t Switch1_PID2Enable;
uint8_t Switch2_ModulationEnable;
uint8_t Switch2_PID1Enable;
uint8_t Switch2_PID2Enable;
uint8_t PIDInputOption;
double Switch1_DACOffset;
double Switch2_DACOffset;
#define V2MV 1000
#define V2MUV 1e6
typedef struct tPID     /// Struktur mit allen wichtigen Variablen des PI-Reglers
{
     double _Ts;
     double _Kp;
     double _Ki;
     double _Kd;
     double _max;
     double _min;
     double _Kaw;
     int _EnKc;
     double _fc;
     double omega;
     double _PIDHold;
     double error_AnWi;
     double error;
     double 	error_1lag;
     double 	error_2lag;
     double    _kt;
     double preout;
     int    En;
     double  a0;
     double  a1;
     double  a2;
     double  aw;
     double ref;
     double outvalue;
}tPID;
tPID InstancePID1;
tPID InstancePID2;
void PID_vInit(tPID*);
void PID_Calc(tPID*,uint8_t, double, double);
float tty1,tty2,tty3,tty4;
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define DAC_ALIGNMENT DAC_ALIGN_12B_R
#define hdac hdac1
#define TIM2PSC 0
#define CTRLFREQUENCY 2.0e5
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);
float PIDInputSWitch(uint8_t, float, float);
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
