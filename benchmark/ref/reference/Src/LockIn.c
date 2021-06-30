/*
 * LogIn.c
 *
 *  Created on: 07.06.2019
 *      Author: zhang
 */
#include <LockIn.h>
#include <math.h>
#include "stm32h7xx_hal.h"
#include "main.h"

void DDS_vInit(float amp, float freq, float fclk,int accumulatorWidth, int LUTGridWidth)
{
	tDDS* 		 pDDS       =   &InstanceDDS;
	float        temp      	=   0;
	signed int 	 SinN      	=   LUTGridWidth;
	float 		 StepNo    	=   (float)(1<<SinN);
	float        StepEnc   	=   (float)(2.0f*PI-2.0f*PI/(StepNo-1.0))/(StepNo-1);
	pDDS->enable            =   1;
	pDDS->amp 				= 	amp;
	pDDS->freq 				= 	freq;
	pDDS->phaseOffset 		= 	100;
	pDDS->fclk 				= 	fclk;
	pDDS->accumulatorWidth 	=	accumulatorWidth;
	pDDS->LUTGridWidth 		= 	LUTGridWidth;
	pDDS->TWSumShift        = 	0;
	pDDS->TWSum             = 	0;
	pDDS->phaseOffset_1ag   = 	INIVALUE;
	pDDS->fclk_Shift_ACCWidth = (pDDS->fclk)/(1<<pDDS-> accumulatorWidth);
	pDDS->pi_Shift_AccWidth   = 10;//(float)(1<<pDDS-> accumulatorWidth)/PI/2.0;
    pDDS->Out               = 	0;
    pDDS->ShiftOut          =	0;
	for(int li=0; li<=(1<<LUTGridWidth)-1; li++)
	{
		LUT_T[li]=li;
		LUT_d[li]=sinf(temp);
		temp = temp+StepEnc;
     }
};

void FIR_Init()
{

	tFIR* sFIR =&InstanceFIR;
	for(int li=0; li<BUFFER_LEN-1; li++)
	{
		sFIR->FIR_ACCUMULATOR[li]=0;
	}
	sFIR->Idx    =0;
	sFIR->SUMME  =0.0f;
	sFIR->FIR_Freq=2000.0f;
}
void DDS_Calc()
{
	tDDS* 		pDDS       =   &InstanceDDS;
	int 		phase_out;
	int 		phase_outShift;
	float 		LUTIndexShift;
	float 		LUTIndex;
	float       Offset=0.5;

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
	     pDDS->ShiftOut =LUTinterp(LUTIndexShift, LUT_T, LUT_d);
	 }
	 else
	 {
		 pDDS->Out=0;
		 pDDS->ShiftOut=0;
	 }
};

float FIR_Calc(float Input,float fclk)
{

	tFIR* sFIR =&InstanceFIR;
	int16_t Idx;
	Idx  =  sFIR->Idx;
	sFIR->SUMME                                 =    sFIR->SUMME-sFIR->FIR_ACCUMULATOR[Idx]+Input;
	sFIR->FIR_ACCUMULATOR[Idx]                  =    Input;
	sFIR->ACCUMLen                              =    round(fclk/(sFIR->FIR_Freq));
	 if (Idx<sFIR->ACCUMLen -1)
		    {
		    	Idx++;
		    }
		    else
		    {
		    	Idx =  0;
		    }
	 sFIR->Idx                                 =     Idx;
	 return(sFIR->SUMME/sFIR->ACCUMLen);
}


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
