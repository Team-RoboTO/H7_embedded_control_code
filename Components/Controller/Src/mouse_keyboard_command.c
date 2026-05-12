#include "mouse_keyboard_command.h"
#include "Remote_Control.h"
#include "stdbool.h"
#include "stm32h7xx_hal.h"
#include "control_utils.h"
#include "robot_config.h"
#include "CRC.h"
//#include "control_std_circ_chassis.h"

  /**************************/
 /*   KEYBOARD VARIABLES   */
/**************************/

fp32 weight_fwd_key;  // Weight of forward movement key (W)
fp32 weight_left_key;  // Weight of left movement key (A)
fp32 weight_bwd_key;  // Weight of backward movement key (S)
fp32 weight_right_key;  // Weight of right movement key (D)
VT13_Info_TypeDef remote_commands;
VT13_Info_TypeDef remote_commands_prev;
  /**************************/
 /*   KEYBOARD FUNCTIONS   */
/**************************/

void mouse_keyboard_commands_update(void) {
	remote_commands_prev = remote_commands;
	
	/* Update data from server via Image_Transmission */
	remote_commands.Mouse.X       = RC_info.Mouse.X;
	remote_commands.Mouse.Y       = RC_info.Mouse.Y;
	remote_commands.Mouse.Z       = RC_info.Mouse.Z;
	remote_commands.Mouse.Press_L = RC_info.Mouse.Press_L;
	remote_commands.Mouse.Press_R = RC_info.Mouse.Press_R;
	remote_commands.Key.V         = RC_info.Key.V;
}

void compute_weights_WASD_keys(float dt) {
	
	mouse_keyboard_commands_update();
	
	/* compute WASD keys weights */
	
	weight_fwd_key 		+= (is_keyboard_key_pressed(KEY_W)) ? dt*10 : - dt*1000;
	weight_left_key 	+= (is_keyboard_key_pressed(KEY_A)) ? dt*10 : - dt*1000;
	weight_bwd_key 		+= (is_keyboard_key_pressed(KEY_S)) ? dt*10 : - dt*1000;
	weight_right_key 	+= (is_keyboard_key_pressed(KEY_D)) ? dt*10 : - dt*1000;
	
	/* saturate WASD keys weights between 0 and 1 */
	weight_fwd_key 		= max(min(weight_fwd_key, 	1.0), 0.0);
	weight_left_key 	= max(min(weight_left_key, 	1.0), 0.0);
	weight_bwd_key 		= max(min(weight_bwd_key, 	1.0), 0.0);
	weight_right_key 	= max(min(weight_right_key, 1.0), 0.0);
}

uint8_t is_keyboard_key_pressed(uint16_t key) {
	
    if (keyboard_keys & key)
        return true;
    
    return false	;
}

uint8_t is_keyboard_prev_key_pressed(uint16_t key) {
	
	if (keyboard_keys_prev & key)
		return true;
	
    return false;
}

uint8_t is_keyboard_key_falling_edge(uint16_t key) {

    if (!is_keyboard_key_pressed(key) && is_keyboard_prev_key_pressed(key))
        return true;
    
    return false;
}

uint8_t is_keyboard_key_raising_edge(uint16_t key) {
    
    if (is_keyboard_key_pressed(key) && !is_keyboard_prev_key_pressed(key))
        return true;
    
    return false;
}

uint8_t is_any_WASD_keyboard_key_pressed(void) {
    
    if (is_keyboard_key_pressed(KEY_W) || is_keyboard_key_pressed(KEY_A) || is_keyboard_key_pressed(KEY_S) || is_keyboard_key_pressed(KEY_D))
        return true;
    
    return false;
}

  /***********************/
 /*   MOUSE FUNCTIONS   */
/***********************/

uint8_t is_mouse_key_pressed(uint8_t key) {
    
    switch (key) {
        
        case MOUSE_LEFT_KEY:
            return (uint8_t) mouse_left_key;
        
        case MOUSE_RIGHT_KEY:
            return (uint8_t) mouse_right_key;
        
        default:
            return false;
    }
}

uint8_t is_mouse_prev_key_pressed(uint8_t key) {
    
    switch (key) {
        
        case MOUSE_LEFT_KEY:
            return (uint8_t) mouse_left_key_prev;
        
        case MOUSE_RIGHT_KEY:
            return (uint8_t) mouse_right_key_prev;
        
        default:
            return false;
    }
}

uint8_t is_mouse_key_falling_edge(uint8_t key) {
    
    if (!is_mouse_key_pressed(key) && is_mouse_prev_key_pressed(key))
        return true;
    
    return false;
}

uint8_t is_mouse_key_raising_edge(uint8_t key) {
    
    if (is_mouse_key_pressed(key) && !is_mouse_prev_key_pressed(key))
        return true;
    
    return false;
}

int16_t yaw_command_mouse_to_remote_controller(fp32 dt) {
    
    // Get mouse velocity
    fp32 x_vel = mouse_lin_vel_x;
    
    // Convert mouse velocity to remote controller tilt
    fp32 remote_controller_left_right_tilt = (- x_vel) * dt * X_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT;
    saturate(&remote_controller_left_right_tilt, 660);
    
    return (int16_t) remote_controller_left_right_tilt;
}

int16_t pitch_command_mouse_to_remote_controller(fp32 dt) {
    
    // Get mouse velocity
    fp32 y_vel = mouse_lin_vel_y;
    
    // Convert mouse velocity to remote controller tilt
    static fp32 remote_controller_bwd_fwd_tilt;
	remote_controller_bwd_fwd_tilt += (- y_vel) * Y_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT * dt;
    saturate(&remote_controller_bwd_fwd_tilt, 660);
    
    return (int16_t) remote_controller_bwd_fwd_tilt;
}

void update_gimbal_references_from_mouse_movements(fp32 *r_yaw, fp32 *r_pitch, fp32 dt) {
    
    // Get mouse velocities and saturate them
    fp32 x_vel = mouse_lin_vel_x;
    fp32 y_vel = mouse_lin_vel_y;
    saturate(&x_vel, SATURATION_MOUSE_LIN_VEL_X);
    saturate(&y_vel, SATURATION_MOUSE_LIN_VEL_Y);
    
    // Update yaw and pitch references
    *r_yaw   -= x_vel * dt * MOUSE_METERS_TO_YAW_RAD;
    *r_pitch -= y_vel * dt * MOUSE_METERS_TO_PITCH_RAD;
}