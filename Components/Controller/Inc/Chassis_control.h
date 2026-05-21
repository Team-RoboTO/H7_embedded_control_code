#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include "robot_config.h"

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_MIT_t chassis;
extern float MIT_kd_base;
extern float MIT_kd;
extern uint8_t is_rotating;

  /********************/
 /*   CONTROL LOOP   */
/********************/
#define GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD 0.0f

void control_loop_chassis(void);

extern float w;

#endif
