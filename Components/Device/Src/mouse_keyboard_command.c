#include "mouse_keyboard_command.h"
#include "control_utils.h"
#include "CRC.h"

  /**************************/
 /*   KEYBOARD VARIABLES   */
/**************************/

float weight_fwd_key;  // Weight of forward movement key (W)
float weight_left_key;  // Weight of left movement key (A)
float weight_bwd_key;  // Weight of backward movement key (S)
float weight_right_key;  // Weight of right movement key (D)

static VT13_Info_TypeDef RC_info_prev;
  /**************************/
 /*   KEYBOARD FUNCTIONS   */
/**************************/

void compute_weights_WASD_keys(float dt) {	
	/* compute WASD keys weights */
	
	weight_fwd_key 		+= (is_keyboard_key_pressed(KEY_W)) ? dt*20 : - dt*1000;
	weight_left_key 	+= (is_keyboard_key_pressed(KEY_A)) ? dt*20 : - dt*1000;
	weight_bwd_key 		+= (is_keyboard_key_pressed(KEY_S)) ? dt*20 : - dt*1000;
	weight_right_key 	+= (is_keyboard_key_pressed(KEY_D)) ? dt*20 : - dt*1000;
	
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

int16_t yaw_command_mouse_to_remote_controller(float dt) {
    
    // Get mouse velocity
    float x_vel = mouse_lin_vel_x;
    
    // Convert mouse velocity to remote controller tilt
    float remote_controller_left_right_tilt = (- x_vel) * dt * X_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT;
    saturate(&remote_controller_left_right_tilt, 660);
    
    return (int16_t) remote_controller_left_right_tilt;
}

int16_t pitch_command_mouse_to_remote_controller(float dt) {
    
    // Get mouse velocity
    float y_vel = mouse_lin_vel_y;
    
    // Convert mouse velocity to remote controller tilt
    static float remote_controller_bwd_fwd_tilt;
	remote_controller_bwd_fwd_tilt += (- y_vel) * Y_MOUSE_METERS_TO_REMOTE_CONTROLLER_TILT * dt;
    saturate(&remote_controller_bwd_fwd_tilt, 660);
    
    return (int16_t) remote_controller_bwd_fwd_tilt;
}

void update_gimbal_references_from_mouse_movements(float *r_yaw, float *r_pitch, float dt) {
    
    // Get mouse velocities and saturate them
    float x_vel = mouse_lin_vel_x;
    float y_vel = mouse_lin_vel_y;
    saturate(&x_vel, SATURATION_MOUSE_LIN_VEL_X);
    saturate(&y_vel, SATURATION_MOUSE_LIN_VEL_Y);
    
    // Update yaw and pitch references
    *r_yaw   -= x_vel * dt * MOUSE_METERS_TO_YAW_RAD;
    *r_pitch -= y_vel * dt * MOUSE_METERS_TO_PITCH_RAD;
}