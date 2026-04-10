#ifndef MOUSE_KEYBOARD_COMMANDS_H
#define MOUSE_KEYBOARD_COMMANDS_H

#include "struct_typedef.h"
#include "robot_config.h"
#include "Remote_Control.h"
#include "Image_Transmission.h"

  /**************************/
 /*   KEYBOARD VARIABLES   */
/**************************/

extern fp32 weight_fwd_key;  // Weight of forward movement key (W)
extern fp32 weight_left_key;  // Weight of left movement key (A)
extern fp32 weight_bwd_key;  // Weight of backward movement key (S)
extern fp32 weight_right_key;  // Weight of right movement key (D)
extern VT13_Info_TypeDef remote_commands;
extern VT13_Info_TypeDef remote_commands_prev;

  /**************************************/
 /*   MOUSE TRACKING HYPERPARAMETERS   */
/**************************************/

#define SATURATION_MOUSE_LIN_VEL_X                  1.0f  // [m/s]
#define SATURATION_MOUSE_LIN_VEL_Y                  1.0f  // [m/s]
#define MOUSE_METERS_TO_YAW_RAD                     100  // [rad/m] If mouse travels 0.01 meters, then yaw has to travel 35*pi/180 rad
#define MOUSE_METERS_TO_PITCH_RAD                   104.720  // [rad/m] If mouse travels 0.005 meters, then yaw has to travel 30*pi/180 rad


#define X_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT    (660/0.010)  // [-/rad] 660 RC tilt corresponds to 4.4 mm
#define Y_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT    (660/0.025)  // [-/rad] 660 RC tilt corresponds to 5.0 mm

#define keyboard_keys		remote_commands.Key.V
#define keyboard_keys_prev		remote_commands_prev.Key.V

  /**************************/
 /*   KEYBOARD KEYS   */
/**************************/


#define KEY_W            ((uint16_t) 1 << 0)
#define KEY_S            ((uint16_t) 1 << 1)
#define KEY_A            ((uint16_t) 1 << 2)
#define KEY_D            ((uint16_t) 1 << 3)
#define KEY_SHIFT        ((uint16_t) 1 << 4)
#define KEY_CTRL         ((uint16_t) 1 << 5)
#define KEY_Q            ((uint16_t) 1 << 6)
#define KEY_E            ((uint16_t) 1 << 7)
#define KEY_R            ((uint16_t) 1 << 8)
#define KEY_F            ((uint16_t) 1 << 9)
#define KEY_G            ((uint16_t) 1 << 10)
#define KEY_Z            ((uint16_t) 1 << 11)
#define KEY_X            ((uint16_t) 1 << 12)
#define KEY_C            ((uint16_t) 1 << 13)
#define KEY_V            ((uint16_t) 1 << 14)
#define KEY_B            ((uint16_t) 1 << 15)

  /**************************/
 /*   KEYBOARD FUNCTIONS   */
/**************************/


/************************************************************************************************************************
	NAME: compute_weights_WASD_keys
	
	DESCRIPTION:
	- for W,A,S,D keys, determines for how long each of them has been pressed consecutively until now.
	- all weights are in range [0,1].
	- each weight reaches the value 1 in 1 second (starting from 0).
	
	ARGUMENTS:
	- dt:		time period (i.e. inverse of frequency) of the code that executes the function
************************************************************************************************************************/
void mouse_keyboard_commands_update(void);

void compute_weights_WASD_keys(float dt);

uint8_t is_keyboard_key_pressed(uint16_t key);
uint8_t is_keyboard_prev_key_pressed(uint16_t key);
uint8_t is_keyboard_key_falling_edge(uint16_t key);
uint8_t is_keyboard_key_raising_edge(uint16_t key);
uint8_t is_any_WASD_keyboard_key_pressed(void);

  /***********************/
 /*   MOUSE KEYS   */
/***********************/

#define MOUSE_LEFT_KEY      1
#define MOUSE_RIGHT_KEY     2

  /***********************/
 /*   MOUSE FUNCTIONS   */
/***********************/

uint8_t is_mouse_key_pressed(uint8_t key);
uint8_t is_mouse_prev_key_pressed(uint8_t key);
uint8_t is_mouse_key_falling_edge(uint8_t key);
uint8_t is_mouse_key_raising_edge(uint8_t key);
int16_t yaw_command_mouse_to_remote_controller(fp32 dt);
int16_t pitch_command_mouse_to_remote_controller(fp32 dt);
void update_gimbal_references_from_mouse_movements(fp32 *r_yaw, fp32 *r_pitch, fp32 dt);


  /******************/
 /*   MOUSE DATA   */
/******************/

#define mouse_lin_vel_x         (remote_commands.Mouse.X * 1e-3)  // Mouse linear velocity along X (horizontal, positive from left to right) [m/s]
#define mouse_lin_vel_y         (remote_commands.Mouse.Y * 1e-3)  // Mouse linear velocity along Y (vertical, positive from up to down) [m/s]
#define mouse_lin_vel_z         (remote_commands.Mouse.Z * 1e-3)  // Mouse linear velocity along Z [m/s]
#define mouse_left_key          remote_commands.Mouse.Press_L
#define mouse_right_key         remote_commands.Mouse.Press_R
#define mouse_left_key_prev     remote_commands_prev.Mouse.Press_L
#define mouse_right_key_prev    remote_commands_prev.Mouse.Press_R

#endif