#ifndef CONTROLLED_SYSTEM_H
#define CONTROLLED_SYSTEM_H

#include "struct_typedef.h"

  /********************************/
 /*   CONTROLLED SYSTEM STRUCT   */
/********************************/

#define MAX_NUM_CONTROLLED_SYSTEM_STATES 4
#define MAX_NUM_CONTROLLED_SYSTEM_INPUTS 4
#define MAX_NUM_CONTROLLED_SYSTEM_OUTPUTS 5

typedef struct controlled_system {
	
	uint8_t n;  // Number of system states
	uint8_t m;  // Number of system inputs
    uint8_t p;  // Number of system outputs
	fp32 x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system states outputs
	fp32 x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous system states outputs
	fp32 u[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Current system inputs
	fp32 u_prev[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Previous system inputs
	fp32 ud[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Current digital system inputs
	fp32 ud_prev[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Previous digital system inputs
	fp32 r_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current references for system states
	fp32 r_x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous references for system states
	fp32 e_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system errors w.r.t. references
	fp32 e_x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous system errors w.r.t. references
	fp32 ei_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system integral errors w.r.t. references
	fp32 ed_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system derivative errors w.r.t. references
    
} controlled_system_t;

#endif