#ifndef CONTROL_STD_CIRC_SHOOTING_H
#define CONTROL_STD_CIRC_SHOOTING_H

#include "robot_config.h"

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t std_circ_shoot_wheels;
extern controlled_system_t std_circ_rev;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_std_circ_shooting(void);
void _control_loop_std_circ_shoot_wheels(void);
void _control_loop_std_circ_rev(void);

#endif