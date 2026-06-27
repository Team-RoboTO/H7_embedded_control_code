#ifndef LIDAR_LIFTER_CONTROL_H   
#define LIDAR_LIFTER_CONTROL_H

#include "robot_config.h"

#if IS_STD || IS_SENTRY

#include "controlled_system.h"

#define LIDAR_CURRENT_TRESHOLD 3000
#define CALIBRATION_SPEED 10 // rad/s
#define SETPOINT_DISTANCE 105.0f

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