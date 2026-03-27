#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

  /***********************/
 /*   ROBOT SELECTION   */
/***********************/

#define IS_STD          1
#define IS_SENTRY       0
#define IS_HERO         0

  /***************************/
 /*   ROBOT CONFIGURATION   */
/***************************/

#define IS_CHASSIS_ENABLED 				    1  // Enable/Disable chassis motors
#define IS_GIMBAL_ENABLED 			     	1  // Enable/Disable gimbal motors
#define IS_SHOOT_WHEELS_ENABLED 	    1  // Enable/Disable shooting wheels motors
#define IS_REV_ENABLED 				      	1  // Enable/Disable REV motor

  /********************/
 /* COMPETITION MODE */
/********************/

#define IS_MATCH_MODE_ENABLED         0  // Enable/Disable match mode (shooting wheels time on)
#define IS_HEAT_ENABLED               0  // shooting limit 
#define IS_POWER_LIMIT_ENABLED        0  // chassis power consumption limit 
#define SUPERCAP_ENABLED              0  // Enable/Disable supercapacitor module

  /********************/
 /*    EVENT MODE    */
/********************/

#define IS_EVENT_MODE_ENABLED         0  // event mode (specific lower velocity)

  /**********************/
 /*    REMOTE CHOICE   */
/**********************/

#define IS_NDJ6_REMOTE                0  // old remote
#define IS_VT13_REMOTE                1  // new remote


  /*****************************/
 /*   CHECK ROBOT SELECTION   */
/*****************************/

#define IS_ROBOT_CONFIG_WRONG \
    IS_STD          < 0 || \
    IS_SENTRY       < 0 || \
    IS_HERO         < 0 || \
    IS_STD          > 1 || \
    IS_SENTRY       > 1 || \
    IS_HERO         > 1 || \
    IS_STD + IS_SENTRY + IS_HERO != 1

// Event mode and shooting cannot be on at the same time
#define CONFLICT_EVENT_SHOOTING \
    (IS_SHOOT_WHEELS_ENABLED + IS_EVENT_MODE_ENABLED) > 1

#endif
