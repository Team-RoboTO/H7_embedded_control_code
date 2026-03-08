#ifndef CONTROLLED_SYSTEM_H
#define CONTROLLED_SYSTEM_H

#include "stdint.h"

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
	float x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system states outputs
	float x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous system states outputs
	float u[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Current system inputs
	float u_prev[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Previous system inputs
	float ud[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Current digital system inputs
	float ud_prev[MAX_NUM_CONTROLLED_SYSTEM_INPUTS];  // Previous digital system inputs
	float r_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current references for system states
	float r_x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous references for system states
	float e_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system errors w.r.t. references
	float e_x_prev[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Previous system errors w.r.t. references
	float ei_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system integral errors w.r.t. references
	float ed_x[MAX_NUM_CONTROLLED_SYSTEM_STATES];  // Current system derivative errors w.r.t. references
    
} controlled_system_t;

#endif