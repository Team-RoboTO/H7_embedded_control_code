#ifndef CONTROL_STD_CIRC_SHOOTING_H
#define CONTROL_STD_CIRC_SHOOTING_H

#include "robot_config.h"
#include "stdbool.h"
#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t shoot_wheels_and_rev;
extern controlled_system_t lidar_lifter;

extern bool unstuck_rev_enabled;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_shooting(void);
void _control_loop_shoot_wheels(void);
void _control_loop_rev(void);
void _control_loop_push(void);
void _control_loop_lidar_lifter(void);

#endif