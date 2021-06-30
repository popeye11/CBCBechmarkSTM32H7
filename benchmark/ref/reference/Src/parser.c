
#include <LockIn.h>
#include"parser.h"
#include"main.h"
#include <math.h>
#include "stm32h7xx_hal.h"
extern uint8_t aTxBuffer[TXBUFFERSIZE];
extern uint8_t aRxBuffer[RXBUFFERSIZE];
extern uint8_t ParserBuffer[PSBUFFERSIZE];
extern float DACOffset;
char *cmdstr[2]   			={
		           	   	   	   "param-set!",
							   "param-ref"
                   	   	   	 };
typedef struct { char *key; int val; } t_paramstruct;
static t_paramstruct lookuptable[] = {
    { "pid1_Ts", IDXPID1_Ts }, 						{ "pid1_kp", IDXPID1_KP }, 						{ "pid1_ki", IDXPID1_KI }, 						{ "pid1_kd", IDXPID1_KD },						{"pid1_out_max",IDXPID1_OUT_MAX},
	{"pid1_out_min",IDXPID1_OUT_MIN},  				{"pid1_kaw",IDXPID1_KAW},       				{"pid1_cutoff_en",IDXPID1_CUTOFF_EN},			{ "pid1_cutoff_freq",IDXPID1_CUTOFF_FREQ},		{ "pid1_hold",IDXPID1_HOLD},
	{"pid1_kt",IDXPID1_KT},           				{"DDS_freq",IDXDDS_FREQ},						{"DDS_phaseOffset",IDXDDS_PHASEOFFSET},			{ "DDS_amp",IDXDDS_AMP},						{"Switch1_DAC_offset",IDXSWICH1_DAC_OFFSET},
	{"FIR_freq",IDXFIR_FREQ	},						{"DDS_enable",IDXDDS_ENABLE},					{"Switch1_DACPID1_EN",IDXSWITCH1_DACPID1_EN},   { "Switch1_DACPID2_EN",IDXSWITCH1_DACPID2_EN},  {"Switch1_offset_En",IDXSWITCH1_OFFSET_EN},
	{"Switch1_DACMod_En",IDXSWITCH1_DACMOD_EN},		{"PID1En",IDXPID1EN},							{"PID_REF",IDXPID_REF},							{ "PIDInputOption",IDXPIDINPUTOPTION},			{"pid2_Ts",IDXPID2_TS},
	{"pid2_kp",IDXPID2_KP},							{"pid2_ki",IDXPID2_KI},							{"pid2_kd",IDXPID2_KD},							{"pid2_out_max",IDXPID2_OUT_MAX},				{"pid2_out_min",IDXPID2_OUT_MIN},
	{"pid2_kaw", IDXPID_KAW },						{"pid2_cutoff_en",IDXPID2_CUTOFF_EN},			{"pid2_cutoff_freq",IDXPID2_CUTOFF_FREQ},		{"pid2_hold",IDXPID2_HOLD},						{"pid2_kt",IDXPID2_KT},
	{"PID2En",IDXPID2EN  },							{"Switch2_DACPID1_EN",IDXSWITCH2_DACPID1_EN},	{"Switch2_DACPID2_EN",IDXSWITCH2_DACPID2_EN},	{"Switch2_offset_En",IDXSWITCH2_OFFSET_EN },	{"Switch2_DACMod_En",IDXSWITCH2_DACMOD_EN},
	{"Switch2_DAC_offset",IDXSWITCH2_DAC_OFFSET}
};
tFIR* 		 sFIR       	= 	&InstanceFIR;
tDDS* 		 sDDS       	=   &InstanceDDS;
tPID* 		 PID1       	=   &InstancePID1;
tPID* 		 PID2       	=   &InstancePID2;
t_paramstruct *sym;
int cmdselect(char *stra[], char *strb[],int16_t paramsize)
{
	int cmpsumintern;
	int cmpvalue;
    int strlen;
    int lj;
    strlen = paramsize-1;
	for(lj=0;lj<=strlen;lj=lj+1)
	{
		cmpvalue=(strcmp(stra[lj],strb[0])==0)? 1: 0;
		cmpsumintern+=cmpvalue*(lj+1);
	}
	return cmpsumintern;
}
int keyfromstring(char *strb[])
{
    int i;
    for (i=0; i < PARAMSIZE; i++) {
    	*sym = lookuptable[i];
        if (strcmp(sym->key,strb[0]) == 0)
        {
            return sym->val;
        }

    }
    return -1;
}

int parser()
{
	int   	PInt;
	int   	PDec;
	float 	FDec;
	char  	Param[20];
	int     status;
	float 	parserOut;
	char 	cmd[20];
	int32_t 	cmdcase;
	int 	paramcase;
	char 	*struart1[1];
	char    *struart2[1];
	float     readvalue;
	float     tty4;
	double 	omega;
	double   Kp;
	double   Kd;
	double   Ki;
	float 		phi;
	float 		phi_lag;
	sscanf(ParserBuffer, "(%s '%s %f)", cmd, Param, &parserOut);
	for (int li=0; li<sizeof(Param)-1;li++)
	{
		if (strncmp(Param[li],')',sizeof(Param)-1)==0)
		  {
			Param[li]='\0';
			}
	}
	struart1[0] = 	cmd;
	struart2[0]	=	Param;
    cmdcase		=	cmdselect(cmdstr,struart1,2);
  //  paramcase   =   cmdselect(paramstr,struart2,42);
    paramcase   =   keyfromstring(struart2);

   switch(cmdcase)
  	{
   		case 1:
   			switch(paramcase)
   			{
   			case IDXPID1_Ts:
   					if (parserOut > PID_TS_MAX)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   			            PID1->_Ts=PID_TS_MAX;
   					}
   					else if (parserOut < PID_TS_MIN)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   						PID1->_Ts=PID_TS_MIN;
   					}
   					else
   					{
   						status =ERRORMSG_NO_ERROR;
   			    		PID1->_Ts=parserOut;
   					}
   			break;
   			case IDXPID1_KP:
   					if (parserOut > PID_KP_MAX)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   						PID1->_Kp=PID_KP_MAX;
   					}
   					else if (parserOut < PID_KP_MIN)
   					{
   					   status = WARNINGMSG_PARAMETER_CLIPPED;
   					   PID1->_Kp=PID_KP_MIN;
   					}
   					else
   					{
   						status =ERRORMSG_NO_ERROR;
   						PID1->_Kp=parserOut;
   					}

   			break;
   			case IDXPID1_KI:
   			   		if (parserOut > PID_KI_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Ki=PID_KI_MAX;
   			   		}
   			   		else if (parserOut < PID_KI_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Ki=PID_KI_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_Ki=parserOut;
   			   		}
   			break;
   			case IDXPID1_KD:
   			   		if (parserOut > PID_KD_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Kd=PID_KD_MAX;
   			   		}
   			   		else if (parserOut < PID_KD_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Kd=PID_KD_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_Kd=parserOut;
   			   		}
   			break;
   			case IDXPID1_OUT_MAX:
   			   		if (parserOut > PID_MAX_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_max=PID_MAX_MAX;
   			   		}
   			   		else if (parserOut < PID_MAX_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_max=PID_MAX_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_max=parserOut;
   			   		}
   			break;
   			case IDXPID1_OUT_MIN:
   			   		if (parserOut > PID_MIN_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_min=PID_MIN_MAX;
   			   		}
   			   		else if (parserOut < PID_MIN_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_min=PID_MIN_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_min=parserOut;
   			   		}
   			break;
   			case IDXPID1_KAW:
   			   		if (parserOut > PID_KAW_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Kaw=PID_KAW_MAX;
   			   		}
   			   		else if (parserOut < PID_KAW_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_Kaw=PID_KAW_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_Kaw=parserOut;
   			   		}
   			break;
   			case IDXPID1_CUTOFF_EN:
   			   		if (parserOut > PID_ENKC_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_EnKc=PID_ENKC_MAX;
   			   		}
   			   		else if (parserOut < PID_ENKC_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_EnKc=PID_ENKC_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_EnKc=parserOut;
   			   		}
   			break;
   			case IDXPID1_CUTOFF_FREQ:
   			   		if (parserOut > PID_FC_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_fc=PID_FC_MAX;
   			   		}
   			   		else if (parserOut < PID_FC_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_fc=PID_FC_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_fc=parserOut;
   			   		}
   			break;
   			case IDXPID1_HOLD:
   			   		if (parserOut > PID_PIDHOLD_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_PIDHold=PID_PIDHOLD_MAX;
   			   		}
   			   		else if (parserOut < PID_PIDHOLD_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_PIDHold=PID_PIDHOLD_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_PIDHold=parserOut;
   			   		}
   			break;
   			case IDXPID1_KT:
   			   		if (parserOut > PID_KT_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_kt=PID_KT_MAX;
   			   		}
   			   		else if (parserOut < PID_KT_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID1->_kt=PID_KT_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID1->_kt=parserOut;
   			   		}
   			break;
   			case IDXDDS_FREQ:
   	   			   		if (parserOut > DDSFREQ_MAX)
   	   			   		{
   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   	   			   	        sDDS->freq=DDSFREQ_MAX;
   	   			   		}
   	   			   		else if (parserOut < DDSFREQ_MIN)
   	   			   		{
   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   	   			   			sDDS->freq=DDSFREQ_MIN;
   	   			   		}
   	   			   		else
   	   			   		{
   	   			   			status =ERRORMSG_NO_ERROR;
   	   			   			sDDS->freq=parserOut;
   	   			   		}
   	   		break;
   			case IDXDDS_PHASEOFFSET://DDS_phaseOffset
					sFIR->SUMME=0;
	   				sFIR->Idx=0;
	   			for(int16_t li=0;li<BUFFER_LEN-1;li++)
	   			{
	   			   sFIR->FIR_ACCUMULATOR[li]= 0;
	   			}
   	   			   		if (parserOut > DDSPHASE_MAX)
   	   			   		{
   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   	   			   			sDDS->phaseOffset=DDSPHASE_MAX;
   	   			   		}
   	   			   		else if (parserOut < DDSPHASE_MIN)
   	   			   		{
   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   	   			   			sDDS->phaseOffset=DDSPHASE_MIN;
   	   			   		}
   	   			   		else
   	   			   		{
   	   			   			status =ERRORMSG_NO_ERROR;
   	   			   			sDDS->phaseOffset=parserOut;
   	   			   		}
   	   		break;
   			case IDXDDS_AMP://DDS_amp
   								sFIR->SUMME=0;
   				   				sFIR->Idx=0;
   				   			for(int16_t li=0;li<BUFFER_LEN-1;li++)
   				   			{
   				   			   sFIR->FIR_ACCUMULATOR[li]= 0;
   				   			}

   									for(int16_t li=0;li<BUFFER_LEN-1;li++)
   			   			   			 {
   			   			   			   sFIR->FIR_ACCUMULATOR[li]= 0;
   			   			   			 }
   			   	   			   		if (parserOut > DDSAMP_MAX)
   			   	   			   		{
   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   	   			   			sDDS->amp=DDSAMP_MAX;
   			   	   			   		}
   			   	   			   		else if (parserOut < DDSAMP_MIN)
   			   	   			   		{
   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   	   			   			sDDS->amp=DDSAMP_MIN;
   			   	   			   		}
   			   	   			   		else
   			   	   			   		{
   			   	   			   			status =ERRORMSG_NO_ERROR;
   			   	   			   			sDDS->amp=parserOut;
   			   	   			   		}
   			break;
   			case IDXSWICH1_DAC_OFFSET://DAC_offset
   			   			   	   			   		if (parserOut > DACOFFSET_MAX)
   			   			   	   			   		{
   			   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   	   			   			Switch1_DACOffset=DACOFFSET_MAX;
   			   			   	   			   		}
   			   			   	   			   		else if (parserOut < DACOFFSET_MIN)
   			   			   	   			   		{
   			   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   	   			   			Switch1_DACOffset=DACOFFSET_MIN;
   			   			   	   			   		}
   			   			   	   			   		else
   			   			   	   			   		{
   			   			   	   			   			status =ERRORMSG_NO_ERROR;
   			   			   	   			   			Switch1_DACOffset=parserOut;
   			   			   	   			   		}
   			break;
   			case IDXFIR_FREQ://FIR_freq
   				sFIR->SUMME=0;
   				sFIR->Idx=0;
   			   			   			   	   			   		if (parserOut > FIRFREQ_MAX)
   			   			   			   	   			   		{
   			   			   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   	   			   			sFIR->FIR_Freq=FIRFREQ_MAX;
   			   			   			   	   			   			for(int16_t li=0;li<BUFFER_LEN-1;li++)
   			   			   			   	   			   			{
   			   			   			   	   			   			 sFIR->FIR_ACCUMULATOR[li]= 0;
   			   			   			   	   			   			}

   			   			   			   	   			   		}
   			   			   			   	   			   		else if (parserOut < FIRFREQ_MIN)
   			   			   			   	   			   		{
   			   			   			   	   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   	   			   			sFIR->FIR_Freq=FIRFREQ_MIN;
   			   			   			   	   			   			for(int16_t li=0;li<BUFFER_LEN-1;li++)
   			   			   			   	   			   			{
   			   			   			   	   			   			 sFIR->FIR_ACCUMULATOR[li]= 0;
   			   			   			   	   			   			}
   			   			   			   	   			   		}
   			   			   			   	   			   		else
   			   			   			   	   			   		{
   			   			   			   	   			   			status =ERRORMSG_NO_ERROR;
   			   			   			   	   			   			sFIR->FIR_Freq=parserOut;
   			   			   			   	   			   			for(int16_t li=0;li<BUFFER_LEN-1;li++)
   			   			   			   	   			   			{
   			   			   			   	   			   			 sFIR->FIR_ACCUMULATOR[li]= 0;
   			   			   			   	   			   			}
   			   			   			   	   			   		}
   			break;
   			case IDXDDS_ENABLE://DDS_enable
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 sDDS->enable=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  sDDS->enable=(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXSWITCH1_DACPID1_EN://PID1Enable
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 Switch1_PID1Enable=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  Switch1_PID1Enable=(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXSWITCH1_DACPID2_EN://PID2Enable
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 Switch1_PID2Enable=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  Switch1_PID2Enable=(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXSWITCH1_OFFSET_EN://offsetEnable
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 Switch1_offsetEnable=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  Switch1_offsetEnable=(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXSWITCH1_DACMOD_EN://offsetEnable
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 Switch1_ModulationEnable=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  Switch1_ModulationEnable=(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXPID1EN://pPID->En
   			   			   			   			   	   		if (parserOut < 0)
   			   			   			   			   	   		{
   			   			   			   			   	   			 status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   			   			   	   			 PID1->En=0;
   			   			   			   			   	   		}
   			   			   			   			   	   		else
   			   			   			   			   	   		{
   			   			   			   			   	   			  status =ERRORMSG_NO_ERROR;
   			   			   			   			   	   			  PID1->En =(int)parserOut;
   			   			   			   			   	   		}
   			break;
   			case IDXPID_REF://PID1_REF
   			   			   									if (parserOut > PID_MIN_MAX)
   			   			   									{
   			   			   										status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   										PID1->ref=PID_MIN_MAX;
   			   			   										PID2->ref=PID_MIN_MAX;
   			   			   									}
   			   			   									else if (parserOut < PID_MIN_MIN)
   			   			   									{
   			   			   										status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			   										PID1->ref=PID_MIN_MIN;
   			   			   										PID2->ref=PID_MIN_MIN;
   			   			   									}
   			   			   									else
   			   			   									{
   			   			   										status =ERRORMSG_NO_ERROR;
   			   			   										PID1->ref=parserOut;
   			   			   										PID2->ref=parserOut;
   			   			   									}
   			break;
   			case IDXPIDINPUTOPTION://PIDInputOption
		   			   	   							if (parserOut < 0)
		   			   	   								{
		   			   	   								status = WARNINGMSG_PARAMETER_CLIPPED;
		   			   	   									PIDInputOption=0;
		   			   	   								}
		   			   	   							else
		   			   	   							{
		   			   	   								status =ERRORMSG_NO_ERROR;
		   			   	   									PIDInputOption =(int)parserOut;
		   			   	   							}
		   	break;
   			case IDXPID2_TS:
   					if (parserOut > PID_TS_MAX)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   			            PID2->_Ts=PID_TS_MAX;
   					}
   					else if (parserOut < PID_TS_MIN)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   						PID2->_Ts=PID_TS_MIN;
   					}
   					else
   					{
   						status =ERRORMSG_NO_ERROR;
   			    		PID2->_Ts=parserOut;
   					}
   			break;
   			case IDXPID2_KP:
   					if (parserOut > PID_KP_MAX)
   					{
   						status = WARNINGMSG_PARAMETER_CLIPPED;
   						PID2->_Kp=PID_KP_MAX;
   					}
   					else if (parserOut < PID_KP_MIN)
   					{
   					   status = WARNINGMSG_PARAMETER_CLIPPED;
   					   PID2->_Kp=PID_KP_MIN;
   					}
   					else
   					{
   						status =ERRORMSG_NO_ERROR;
   						PID2->_Kp=parserOut;
   					}

   			break;
   			case IDXPID2_KI:
   			   		if (parserOut > PID_KI_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Ki=PID_KI_MAX;
   			   		}
   			   		else if (parserOut < PID_KI_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Ki=PID_KI_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_Ki=parserOut;
   			   		}
   			break;
   			case IDXPID2_KD:
   			   		if (parserOut > PID_KD_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Kd=PID_KD_MAX;
   			   		}
   			   		else if (parserOut < PID_KD_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Kd=PID_KD_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_Kd=parserOut;
   			   		}
   			break;
   			case IDXPID2_OUT_MAX:
   			   		if (parserOut > PID_MAX_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_max=PID_MAX_MAX;
   			   		}
   			   		else if (parserOut < PID_MAX_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_max=PID_MAX_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_max=parserOut;
   			   		}
   			break;
   			case IDXPID2_OUT_MIN:
   			   		if (parserOut > PID_MIN_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_min=PID_MIN_MAX;
   			   		}
   			   		else if (parserOut < PID_MIN_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_min=PID_MIN_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_min=parserOut;
   			   		}
   			break;
   			case IDXPID_KAW:
   			   		if (parserOut > PID_KAW_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Kaw=PID_KAW_MAX;
   			   		}
   			   		else if (parserOut < PID_KAW_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_Kaw=PID_KAW_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_Kaw=parserOut;
   			   		}
   			break;
   			case IDXPID2_CUTOFF_EN:
   			   		if (parserOut > PID_ENKC_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_EnKc=PID_ENKC_MAX;
   			   		}
   			   		else if (parserOut < PID_ENKC_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_EnKc=PID_ENKC_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_EnKc=parserOut;
   			   		}
   			break;
   			case IDXPID2_CUTOFF_FREQ:
   			   		if (parserOut > PID_FC_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_fc=PID_FC_MAX;
   			   		}
   			   		else if (parserOut < PID_FC_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_fc=PID_FC_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_fc=parserOut;
   			   		}
   			break;
   			case IDXPID2_HOLD:
   			   		if (parserOut > PID_PIDHOLD_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_PIDHold=PID_PIDHOLD_MAX;
   			   		}
   			   		else if (parserOut < PID_PIDHOLD_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_PIDHold=PID_PIDHOLD_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_PIDHold=parserOut;
   			   		}
   			break;
   			case IDXPID2_KT:
   			   		if (parserOut > PID_KT_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_kt=PID_KT_MAX;
   			   		}
   			   		else if (parserOut < PID_KT_MIN)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->_kt=PID_KT_MIN;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->_kt=parserOut;
   			   		}
   			break;
   			case IDXPID2EN://pPID->En
   			   		if (parserOut < 0)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			PID2->En=0;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			PID2->En =(int)parserOut;
   			   		}
   			break;
   			case IDXSWITCH2_DACPID1_EN://PID2Enable
   			   		if (parserOut < 0)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			Switch2_PID1Enable=0;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			Switch2_PID1Enable=(int)parserOut;
   			   		}
   			break;
   			case IDXSWITCH2_DACPID2_EN://PID2Enable
   			   		if (parserOut < 0)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			Switch2_PID2Enable=0;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			Switch2_PID2Enable=(int)parserOut;
   			   		}
   			break;
   			case IDXSWITCH2_OFFSET_EN://offsetEnable
   			   		if (parserOut < 0)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			Switch2_offsetEnable=0;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			Switch2_offsetEnable=(int)parserOut;
   			   		}
   			break;
   			case IDXSWITCH2_DACMOD_EN://offsetEnable
   			   		if (parserOut < 0)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			Switch2_ModulationEnable=0;
   			   		}
   			   		else
   			   		{
   			   			status =ERRORMSG_NO_ERROR;
   			   			Switch2_ModulationEnable=(int)parserOut;
   			   		}
   			break;
   			case IDXSWITCH2_DAC_OFFSET://DAC_offset
   			   		if (parserOut > DACOFFSET_MAX)
   			   		{
   			   			status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			Switch2_DACOffset=DACOFFSET_MAX;
   			   		}
   			   		else if (parserOut < DACOFFSET_MIN)
   			   		{
   			   			  status = WARNINGMSG_PARAMETER_CLIPPED;
   			   			  Switch2_DACOffset=DACOFFSET_MIN;
   			   		}
   			   		else
   			   		{
   			   			 status =ERRORMSG_NO_ERROR;
   			   			 Switch2_DACOffset=parserOut;
   			   		}
   			default:
   				status=ERRORMSG_NOSUCH_PARAMETER;
   			}
   			for(int li=0; li<TXBUFFERSIZE-1;li++)
   			{
   				aTxBuffer[li]='\0';
   			}
   			itoa(status,aTxBuffer,10);
   			strcat( aTxBuffer, "\n> " );
   			switch(PID1->_EnKc)
   					{
   					case 1:
   						PID1->omega =PID1->_fc*2.0*M_PI;
   						Kp=PID1-> _Kp;
   						Kd=PID1-> _Kd/V2MUV;
   						Ki=PID1-> _Ki*V2MV;
   						PID1->a0 = (Kp*(1+PID1->omega*PID1->_Ts)+Kd/PID1->_Ts*(1+PID1->omega*PID1->_Ts)+Ki*PID1->_Ts);
   						// e(k-1) term
   						PID1->a1= -(Kp+Kd/PID1->_Ts*(2.0+PID1->omega*PID1->_Ts));
   						// e(k-2) term
   						PID1->a2 =Kd/PID1->_Ts;
   						// anti-windup term
   						PID1->aw   =PID1->_Kaw*PID1->_Ts;
   					break;
   					case 0:
   					default:
   						PID1->omega =0;
   					   	Kp=PID1-> _Kp;
   					   	Kd=PID1-> _Kd/V2MUV;
   					   	Ki=PID1-> _Ki*V2MV;
   					   	PID1->a0 = Kp+Kd/PID1->_Ts+Ki*PID1->_Ts;
   						// e(k-1) term
   					   	PID1->a1 = -(Kp+Kd/PID1->_Ts*2.0);
   						// e(k-2) term
   					   	PID1->a2 =Kd/PID1->_Ts;
   						// anti-windup term
   					   	PID1->aw   =PID1->_Kaw*PID1->_Ts;
   						// Calculate total output
   						//output=Eout_1lag;
   					}
   			switch(PID2->_EnKc)
   			   					{
   			   					case 1:
   			   						PID2->omega =PID2->_fc*2.0*M_PI;
   			   						Kp=PID2-> _Kp;
   			   						Kd=PID2-> _Kd/V2MUV;
   			   						Ki=PID2-> _Ki*V2MV;
   			   						PID2->a0 = (Kp*(1+PID2->omega*PID2->_Ts)+Kd/PID2->_Ts*(1+PID2->omega*PID2->_Ts)+Ki*PID2->_Ts);
   			   						// e(k-1) term
   			   						PID2->a1= -(Kp+Kd/PID2->_Ts*(2.0+PID2->omega*PID2->_Ts));
   			   						// e(k-2) term
   			   						PID2->a2 =Kd/PID2->_Ts;
   			   						// anti-windup term
   			   						PID2->aw   =PID2->_Kaw*PID2->_Ts;
   			   					break;
   			   					case 0:
   			   					default:
   			   						PID2->omega =0;
   			   					   	Kp=PID2-> _Kp;
   			   					   	Kd=PID2-> _Kd/V2MUV;
   			   					   	Ki=PID2-> _Ki*V2MV;
   			   					   	PID2->a0 = Kp+Kd/PID2->_Ts+Ki*PID2->_Ts;
   			   						// e(k-1) term
   			   					   	PID2->a1 = -(Kp+Kd/PID2->_Ts*2.0);
   			   						// e(k-2) term
   			   					   	PID2->a2 =Kd/PID2->_Ts;
   			   						// anti-windup term
   			   					   	PID2->aw   =PID2->_Kaw*PID2->_Ts;
   			   						// Calculate total output
   			   						//output=Eout_1lag;
   			   					}
   		phi               			=     sDDS->phaseOffset/180*PI;
   		phi_lag                     =     sDDS->phaseOffset_1ag/180*PI;
   		sDDS->TW                	=      round((sDDS->freq)/sDDS->fclk_Shift_ACCWidth);            	//      tunning word relative to frequency  round(freq/(fclk/2^N))
   		sDDS->DetTW             	=      round((phi-phi_lag)*(1<<sDDS-> accumulatorWidth)/PI/2.0f);                 			  	//      tunning word relative to phase shift   round(phi/2/pi*2^N)
   		break;
   		case 2:

   			switch(paramcase)
   			{
   				case IDXPID1_Ts:
   						readvalue=PID1->_Ts;
   	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   	   		   			{
   	   		   				aTxBuffer[li]='\0';
   	   		   			}
   	   				sprintf(aTxBuffer,"%.7f",readvalue);
   	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_KP:
   					    readvalue=PID1->_Kp;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_KI:
   						readvalue=PID1->_Ki;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_KD:
   						readvalue=PID1->_Kd;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_OUT_MAX:
   						readvalue=PID1->_max;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_OUT_MIN:
   						readvalue=PID1->_min;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_KAW:
   						readvalue=PID1->_Kaw;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_CUTOFF_EN:
   						readvalue=PID1->_EnKc;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_CUTOFF_FREQ:
   						readvalue=PID1->_fc;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_HOLD:
   						readvalue=PID1->_PIDHold;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1_KT:
   						readvalue=PID1->_kt;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXDDS_FREQ:
   						readvalue=sDDS->freq;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXDDS_PHASEOFFSET:
   						readvalue=sDDS->phaseOffset;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXDDS_AMP:
   						readvalue=sDDS->amp;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWICH1_DAC_OFFSET:
   						readvalue=	Switch1_DACOffset;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXFIR_FREQ:
   				   		readvalue=	sFIR->FIR_Freq;
   					for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   {
   					   		aTxBuffer[li]='\0';
   					   }
   					sprintf(aTxBuffer,"%.7f",readvalue);
   					strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXDDS_ENABLE:
   				   		readvalue=	sDDS->enable;
   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   		{
   				   			aTxBuffer[li]='\0';
   				   		}
   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH1_DACPID1_EN:
   				   		readvalue=Switch1_PID1Enable;
   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   		{
   				   			aTxBuffer[li]='\0';
   				   		}
   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH1_DACPID2_EN:
   				   		readvalue=Switch1_PID2Enable;
   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   		{
   				   			aTxBuffer[li]='\0';
   				   		}
   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH1_OFFSET_EN:
   				   		readvalue=Switch1_offsetEnable;
   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   		{
   				   			aTxBuffer[li]='\0';
   				   		}
   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH1_DACMOD_EN:
   				   		readvalue=Switch1_ModulationEnable;
   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   		{
   				   			aTxBuffer[li]='\0';
   				   		}
   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID1EN:
   				   				   		readvalue=PID1->En;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID_REF:
   				   				   		readvalue=PID1->ref;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPIDINPUTOPTION:
   				   				   		readvalue=PIDInputOption;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_TS:
   				   						readvalue=PID2->_Ts;
   				   	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   	   		   			{
   				   	   		   				aTxBuffer[li]='\0';
   				   	   		   			}
   				   	   				sprintf(aTxBuffer,"%.7f",readvalue);
   				   	   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_KP:
   				   					readvalue=PID2->_Kp;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_KI:
   				   					readvalue=PID2->_Ki;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_KD:
   				   					readvalue=PID2->_Kd;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_OUT_MAX:
   				   					readvalue=PID2->_max;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_OUT_MIN:
   				   					readvalue=PID2->_min;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID_KAW:
   				   					readvalue=PID2->_Kaw;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_CUTOFF_EN:
   				   					readvalue=PID2->_EnKc;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_CUTOFF_FREQ:
   				   					readvalue=PID2->_fc;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_HOLD:
   				   					readvalue=PID2->_PIDHold;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXPID2_KT:
   				   					readvalue=PID2->_kt;
   					   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   					   		   			{
   					   		   				aTxBuffer[li]='\0';
   					   		   			}
   					   				sprintf(aTxBuffer,"%.7f",readvalue);
   					   				strcat( aTxBuffer, "\n> " );
   				break;

   				case IDXPID2EN:
   				   				   		readvalue=PID2->En;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH2_DACPID1_EN:
   				   				   		readvalue=Switch2_PID1Enable;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH2_DACPID2_EN:
   				   				   	readvalue=Switch2_PID2Enable;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH2_OFFSET_EN:
   				   				   	readvalue=Switch2_offsetEnable;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH2_DACMOD_EN:
   				   				   	readvalue=Switch2_ModulationEnable;
   				   				   	for(int li=0; li<TXBUFFERSIZE-1;li++)
   				   				   		{
   				   				   			aTxBuffer[li]='\0';
   				   				   		}
   				   				   	sprintf(aTxBuffer,"%.7f",readvalue);
   				   				   	strcat( aTxBuffer, "\n> " );
   				break;
   				case IDXSWITCH2_DAC_OFFSET:
   						readvalue=	Switch2_DACOffset;
	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
	   		   			{
	   		   				aTxBuffer[li]='\0';
	   		   			}
	   				sprintf(aTxBuffer,"%.7f",readvalue);
	   				strcat( aTxBuffer, "\n> " );
   				break;
   				default:
   					status =ERRORMSG_NOSUCH_PARAMETER;
   	   				for(int li=0; li<TXBUFFERSIZE-1;li++)
   	   		   			{
   	   		   				aTxBuffer[li]='\0';
   	   		   			}
   	   				sprintf(aTxBuffer,"%d",status);
   	   				strcat( aTxBuffer, "\n> " );
   			}
   		break;

   		default:
   			status=ERRORMSG_UNKNOWN_FUNCTION;
   					for(int li=0; li<TXBUFFERSIZE-1;li++)
   		   	   		   	{
   		   	   		   		aTxBuffer[li]='\0';
   		   	   		   	}
   		   	   		sprintf(aTxBuffer,"%d",status);
   		   	   		strcat( aTxBuffer, "\n> " );
   	}
   return(status);
}
