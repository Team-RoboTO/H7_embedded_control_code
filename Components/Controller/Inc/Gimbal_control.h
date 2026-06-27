#ifndef GIMBAL_CONTROL_H
#define GIMBAL_CONTROL_H

#include "robot_config.h"
#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t gimbal;
extern float KD_yaw;
extern float KD_pitch;
extern float KP_pitch;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_gimbal(void);


#endif