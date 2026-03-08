#ifndef STATE_MACHINE_STD_CIRC_H
#define STATE_MACHINE_STD_CIRC_H

#include "robot_config.h"

#include "stdint.h"

  /**************/
 /*   STATES   */
/**************/

// Remote commands
#define COMMANDS_STOP                   0
#define COMMANDS_REMOTE_CONTROLLER      1
#define COMMANDS_KEYBOARD_MOUSE         2

// Chassis
#define CHASSIS_FOLLOW_GIMBAL           0
#define CHASSIS_CONTIGUOUS_ROTATION     1

// Gimbal
#define GIMBAL_MANUAL_AIM               0
#define GIMBAL_AUTO_AIM                 1

// Shoot wheels
#define SHOOT_WHEELS_STOP               0
#define SHOOT_WHEELS_SPIN               1

// Rev
#define REV_STOP                        0
#define REV_SINGLE_SHOOTING             1
#define REV_TRIPLE_SHOOTING             2
#define REV_MULTIPLE_SHOOTING           3
#define REV_UNSTUCK                     4

// State variables
extern uint8_t state_remote_commands;
extern uint8_t state_chassis;
extern uint8_t state_gimbal;
extern uint8_t state_shoot_wheels;
extern uint8_t state_rev;
extern uint8_t state_remote_commands_prev;
extern uint8_t state_chassis_prev;
extern uint8_t state_gimbal_prev;
extern uint8_t state_shoot_wheels_prev;
extern uint8_t state_rev_prev;

  /********************************/
 /*   SHOOT WHEELS SPIN STRUCT   */
/********************************/

typedef struct shoot_wheels_spin {
    
    int16_t threshold_rc_wheel_released;  // Threshold below which the RC wheel is considered to be released (in range [0, 660])
    float timestamp_last_shoot_command;  // Timestamp of last shoot command [s]
    float time_without_shoot_commands_before_stopping_shoot_wheels;  // Time to elapse from last shoot command before stopping the shoot wheels [s]
    
} shoot_wheels_spin_t;

extern shoot_wheels_spin_t shoot_wheels_spin;

  /***********************/
 /*   REV SPIN STRUCT   */
/***********************/

typedef struct rev_spin {
    
    float timestamp_last_shoot_command;  // Timestamp of last shoot command [s]
    float time_threshold_hold_mouse_key_multiple_shooting;  // Threshold time of holding mouse keys after which multiple shooting is triggered [s]
    
} rev_spin_t;

extern rev_spin_t rev_spin;

  /************************/
 /*   STATES FUNCTIONS   */
/************************/

void robot_states_update_state_machine(void);
uint8_t _state_machine_remote_commands();
uint8_t _state_machine_chassis();
uint8_t _state_machine_chassis_remote_controller();
uint8_t _state_machine_chassis_keyboard_mouse();
uint8_t _state_machine_gimbal();
uint8_t _state_machine_gimbal_remote_controller();
uint8_t _state_machine_gimbal_keyboard_mouse();
uint8_t _state_machine_shoot_wheels();
uint8_t _state_machine_shoot_wheels_remote_controller();
uint8_t _state_machine_shoot_wheels_keyboard_mouse();
uint8_t _state_machine_rev();
uint8_t _state_machine_rev_remote_controller();
uint8_t _state_machine_rev_keyboard_mouse();

#endif

