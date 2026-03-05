#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "robot_config.h"

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t chassis;
extern fp32 power_cons;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_chassis(void);

#endif
