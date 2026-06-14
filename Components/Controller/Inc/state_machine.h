#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "robot_config.h"
#include "stdint.h"
#include "stdbool.h"

  /**************/
 /*   STATES   */
/**************/

// Remote commands
#define COMMANDS_STOP                0
#define COMMANDS_REMOTE_CONTROLLER   1
#define COMMANDS_KEYBOARD_MOUSE      2
#define COMMANDS_AUTONOMUS           3

// Chassis
#define CHASSIS_FOLLOW_GIMBAL        0
#define CHASSIS_CONTIGUOUS_ROTATION  1

// Gimbal
#define GIMBAL_MANUAL_AIM            0
#define GIMBAL_AUTO_AIM              1

// Shoot wheels
#define SHOOT_WHEELS_STOP            0
#define SHOOT_WHEELS_SPIN            1

// Rev
#define REV_STOP                     0
#define REV_SINGLE_SHOOTING          1
#define REV_MULTIPLE_SHOOTING        2
#define REV_UNSTUCK                  3

// Push (Hero only)
#define PUSH_STOP                    0
#define PUSH_SINGLE_SHOOTING         1
#define PUSH_MULTIPLE_SHOOTING       2
#define PUSH_UNSTUCK                 3

// Lidar lifter
#define LIDAR_DOWN                   0
#define LIDAR_UP                     1

  /********************************/
 /*   SHOOT WHEELS SPIN CONFIG   */
/********************************/

typedef struct {
    int16_t    threshold_rc_wheel_released;         	 // RC wheel dead-zone threshold [0, 660]
    uint32_t   timestamp_last_shoot_command;        	 // Timestamp of last shoot command [ms]
    uint32_t   time_before_stopping_wheels;	          // Spin-down delay after last command [ms]
} shoot_wheels_spin_t;

  /***************************/
 /*   REV / PUSH CONFIG     */
/***************************/

typedef struct {
    uint32_t    timestamp_last_shoot_command;          // Timestamp of last shoot command [ms]
    uint32_t    time_rev_locked;                       // HAL tick at which stall was first detected [ms]
	  uint32_t    time_unstuck;                          // HAL tick at which stall was first detected [ms]
		uint16_t    shooting_frequency;                    // Shooting frequency [hz]
	  bool        stuck_state;                           // Is rev stucked                
} rev_spin_t;

typedef struct {
    float timestamp_last_shoot_command;                     // Timestamp of last shoot command [s]
    float time_threshold_hold_mouse_key_multiple_shooting;  // Hold duration to enter continuous fire [s]
    float time_rev_locked;                                  // HAL tick at which stall was first detected [ms]
} push_spin_t;

  /******************************/
 /*   BARREL HEAT MANAGEMENT   */
/******************************/

typedef struct {
    float    heat_limit;          	 // Maximum heat allowed by Referee System [heat units]
    float    current_heat;        	 // Current heat tracked locally [heat units]
    float    cooling_rate;        	 // Heat shed per second [heat units/s]
    float    heat_per_projectile; 	 // Heat added per projectile (17 mm = 10, 42 mm = 100)
    uint16_t safe_threshold;      	 // Shoot-block margin below heat_limit (e.g. 2 × heat_per_projectile)
    float    last_shooting_position; // Rev encoder position at last logged shot [rad]
    float    last_cool_time;         // HAL tick of last cooling update [ms]
} barrel_heat_management_t;

void _update_barrel_heat_logic(void);

  /************************/
 /*   STATE FUNCTIONS    */
/************************/

void    robot_states_update_state_machine(void);

extern uint8_t state_remote_commands;
uint8_t _state_machine_remote_commands(void);

extern uint8_t state_chassis;
uint8_t _state_machine_chassis(void);
uint8_t _state_machine_chassis_remote_controller(void);
uint8_t _state_machine_chassis_keyboard_mouse(void);
uint8_t _state_machine_chassis_autonomus(void);

extern uint8_t state_gimbal;
uint8_t _state_machine_gimbal(void);
uint8_t _state_machine_gimbal_remote_controller(void);
uint8_t _state_machine_gimbal_keyboard_mouse(void);
uint8_t _state_machine_gimbal_autonomus(void);

extern uint8_t state_shoot_wheels;
uint8_t _state_machine_shoot_wheels(void);
uint8_t _state_machine_shoot_wheels_remote_controller(void);
uint8_t _state_machine_shoot_wheels_keyboard_mouse(void);
uint8_t _state_machine_shoot_wheels_autonomus(void);

extern uint8_t state_rev;
uint8_t _state_machine_rev(void);
uint8_t _state_machine_rev_remote_controller(void);
uint8_t _state_machine_rev_keyboard_mouse(void);
uint8_t _state_machine_rev_autonomus(void);

#if IS_STD || IS_SENTRY
	extern uint8_t state_lidar_lifter;
	uint8_t _state_machine_lidar_lifter(void);
	
#elif IS_HERO
	extern uint8_t state_push;
	uint8_t _state_machine_push(void);
	uint8_t _state_machine_push_remote_controller(void);
	uint8_t _state_machine_push_keyboard_mouse(void);
	uint8_t _state_machine_push_autonomus(void);
#endif

#endif /* STATE_MACHINE_H */