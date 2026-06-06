#ifndef SHOOTING_CONTROL_H
#define SHOOTING_CONTROL_H

#include "robot_config.h"
#include "stdbool.h"
#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t shoot_wheels;
extern controlled_system_t rev_and_push;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_shooting(void);
void _shooting_control_init(void);
void _control_loop_shoot_wheels(void);
void _control_loop_rev(void);
void _control_loop_push(void);

extern float is_on_reset;

#endif