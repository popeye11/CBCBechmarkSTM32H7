/*
 * parser.h
 *
 *  Created on: 14.05.2019
 *      Author: zhang
 */

#ifndef PARSER_H_
#define PARSER_H_
#include "main.h"
#define PID_TS_MAX 	1.0
#define PID_TS_MIN     1e-6
#define PID_KP_MAX     1000
#define PID_KP_MIN     0
#define PID_KI_MAX     1000
#define PID_KI_MIN     0
#define PID_KD_MAX 	1000
#define PID_KD_MIN 	0
#define PID_MAX_MAX    3.3
#define PID_MAX_MIN    -3.3
#define PID_MIN_MAX    3.3
#define PID_MIN_MIN    -3.3
#define PID_KAW_MAX    1000
#define PID_KAW_MIN    0
#define PID_ENKC_MAX   1
#define PID_ENKC_MIN   0
#define PID_FC_MAX     1000
#define PID_FC_MIN     0
#define PID_PIDHOLD_MAX  1
#define PID_PIDHOLD_MIN  0
#define PID_KT_MAX     1000
#define PID_KT_MIN     -1000
#define DDSFREQ_MAX       200e3
#define DDSFREQ_MIN       0
#define DDSPHASE_MIN      -360
#define DDSPHASE_MAX      360
#define DDSAMP_MIN        0
#define DDSAMP_MAX        3.3/2
#define DACOFFSET_MAX     3.3
#define DACOFFSET_MIN     -3.3
#define FIRFREQ_MAX       1e5
#define FIRFREQ_MIN       0
#define ERRORMSG_INVALID_ARGUMENT 		-1
#define ERRORMSG_NOSUCH_PARAMETER 		-3
#define WARNINGMSG_PARAMETER_CLIPPED 	2
#define ERRORMSG_UNKNOWN_FUNCTION 		-5
#define ERRORMSG_NO_ERROR               0
#define IDXPID1_Ts       		1
#define IDXPID1_KP				2
#define IDXPID1_KI				3
#define IDXPID1_KD				4
#define IDXPID1_OUT_MAX   		5
#define IDXPID1_OUT_MIN			6
#define IDXPID1_KAW				7
#define IDXPID1_CUTOFF_EN		8
#define IDXPID1_CUTOFF_FREQ		9
#define IDXPID1_HOLD		    10
#define IDXPID1_KT		        11
#define IDXDDS_FREQ		        12
#define IDXDDS_PHASEOFFSET		13
#define IDXDDS_AMP				14
#define IDXSWICH1_DAC_OFFSET	15
#define IDXFIR_FREQ				16
#define IDXDDS_ENABLE			17
#define IDXSWITCH1_DACPID1_EN	18
#define IDXSWITCH1_DACPID2_EN	19
#define IDXSWITCH1_OFFSET_EN	20
#define IDXSWITCH1_DACMOD_EN	21
#define IDXPID1EN				22
#define IDXPID_REF				23
#define IDXPIDINPUTOPTION		24
#define IDXPID2_TS              25
#define IDXPID2_KP              26
#define IDXPID2_KI				27
#define IDXPID2_KD              28
#define IDXPID2_OUT_MAX         29
#define IDXPID2_OUT_MIN         30
#define IDXPID_KAW              31
#define IDXPID2_CUTOFF_EN       32
#define IDXPID2_CUTOFF_FREQ     33
#define IDXPID2_HOLD            34
#define IDXPID2_KT				35
#define IDXPID2EN               36
#define IDXSWITCH2_DACPID1_EN   37
#define IDXSWITCH2_DACPID2_EN   38
#define IDXSWITCH2_OFFSET_EN    39
#define IDXSWITCH2_DACMOD_EN	40
#define IDXSWITCH2_DAC_OFFSET	41

char *cmdstr[2];
char *paramstr[PARAMSIZE];
int cmdselect(char *stra[], char *strb[],int16_t paramsize);
int parser();



#endif /* PARSER_H_ */
