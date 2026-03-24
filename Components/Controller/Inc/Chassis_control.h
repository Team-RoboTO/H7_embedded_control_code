#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "robot_config.h"

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_MIT_t chassis;

  /********************/
 /*   CONTROL LOOP   */
/********************/
#define GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD 1000.0f

void control_loop_chassis(void);

#endif
