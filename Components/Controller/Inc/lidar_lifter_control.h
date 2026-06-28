#ifndef LIDAR_LIFTER_CONTROL_H   
#define LIDAR_LIFTER_CONTROL_H

#include "robot_config.h"

#if IS_STD || IS_SENTRY

#include "controlled_system.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

extern controlled_system_t lidar_lifter;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_lidar_lifter(void);

#endif
#endif