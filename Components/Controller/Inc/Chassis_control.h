#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "robot_config.h"

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_MIT_t chassis;
extern float MIT_kd;

  /********************/
 /*   CONTROL LOOP   */
/********************/
#define GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD -0.124877453f

void control_loop_chassis(void);

#endif
