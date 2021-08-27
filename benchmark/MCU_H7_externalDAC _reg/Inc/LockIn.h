/*
 * LogIn.h
 *
 *  Created on: 07.06.2019
 *      Author: zhang
 */

#ifndef LOCKIN_H_
#define LOCKIN_H_
#include "stm32h7xx_hal.h"
#define INIVALUE 			0
#define ACCUMULATORWIDTH    16
#define LUTGRIDWIDTH        10
#define BUFFER_LEN    200
#define PI          		3.141592
#define defrootTerm_LEN 	1024
typedef struct _tDDS
{
  int   enable;
  float amp;
  float freq;
  float phaseOffset;
  float phaseOffset_1ag;
  float fclk;
  int accumulatorWidth;
  int LUTGridWidth;
  int TWSum;
  int TWSumShift;
  float fclk_Shift_ACCWidth;
  float pi_Shift_AccWidth;
  float Out;
  float ShiftOut;
  float phasedlag;
  float phaseround;
  int	 TW;
  int	 DetTW;
}tDDS;
tDDS InstanceDDS;
float FIR_ACCUMULATOR[BUFFER_LEN];
int   FIR_ACCUMLen;
float   FIR_SUM;
int   FIR_IDX;

typedef struct _tFIR
{
	float FIR_ACCUMULATOR[BUFFER_LEN];
	int ACCUMLen;
	float SUMME;
	int Idx;
	float FIR_Freq;
}tFIR;
tFIR InstanceFIR;
//tFIR* sFIR =&InstanceFIR;
signed int LUT_T[defrootTerm_LEN];
float      LUT_d[defrootTerm_LEN];
void  DDS_vInit(float , float, float, int , int);
void  DDS_Calc();// ,  float , float , float );
void  FIR_Init();
float  FIR_Calc(float,float);
float LUTinterp(float, signed int*, float *) ;
float LUTinterp2(float, signed int*, float *) ;
#endif /* LOCKIN_H_ */
