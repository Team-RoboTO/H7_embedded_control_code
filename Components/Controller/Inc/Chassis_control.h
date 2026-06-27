#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "robot_config.h"
#include "stdbool.h"
#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_MIT_t chassis;
extern float MIT_kd;
extern bool is_rotating;

  /********************/
 /*   CONTROL LOOP   */
/********************/
#define LPF_VEL_ALPHA 0.85f


void control_loop_chassis(void);

extern float w, vx, vy;

#endif
