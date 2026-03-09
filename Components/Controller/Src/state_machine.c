#include "state_machine.h"

#include "Remote_Control.h"
#include "stm32h7xx_hal.h"
#include "math_utils.h"
#include <stdlib.h>
#include "Motor.h"

  /**************/
 /*   STATES   */
/**************/

uint8_t state_remote_commands = COMMANDS_STOP;
uint8_t state_chassis = CHASSIS_FOLLOW_GIMBAL;
uint8_t state_gimbal = GIMBAL_MANUAL_AIM;
uint8_t state_shoot_wheels = SHOOT_WHEELS_STOP;
uint8_t state_rev = REV_STOP;
uint8_t state_remote_commands_prev;
uint8_t state_chassis_prev;
uint8_t state_gimbal_prev;
uint8_t state_shoot_wheels_prev;
uint8_t state_rev_prev;
float time_rev_locked = 0;

  /********************************/
 /*   SHOOT WHEELS SPIN STRUCT   */
/********************************/

shoot_wheels_spin_t shoot_wheels_spin = {
    
    .threshold_rc_wheel_released = 100,  // in range [0, 660]
    .timestamp_last_shoot_command = 0,  // [s]
    .time_without_shoot_commands_before_stopping_shoot_wheels = 1.0f  // [s]
};

  /***********************/
 /*   REV SPIN STRUCT   */
/***********************/

rev_spin_t rev_spin = {
    
    .timestamp_last_shoot_command = 0,  // [s]
    .time_threshold_hold_mouse_key_multiple_shooting = 0.2  // [s]
};

  /************************/
 /*   STATES FUNCTIONS   */
/************************/

void robot_states_update_state_machine() {
    
    // Update previous states
    state_remote_commands_prev = state_remote_commands;
    state_chassis_prev = state_chassis;
    state_shoot_wheels_prev = state_shoot_wheels;
    state_rev_prev = state_rev;
    
    // Remote commands
    state_remote_commands = _state_machine_remote_commands();
    
    // Chassis
    state_chassis = _state_machine_chassis();
    
    // Gimbal
    state_gimbal = _state_machine_gimbal();
    
    // Shoot wheels
    state_shoot_wheels = _state_machine_shoot_wheels();
    
    // Rev
    state_rev = _state_machine_rev();
}

uint8_t _state_machine_remote_commands() {
    
    switch(remote_ctrl.rc.s[0]) {
        
        case 2:
            // Stop commands
            return COMMANDS_STOP;
				
				case 1:
            // Get commands from remote controller
            return COMMANDS_REMOTE_CONTROLLER;
        
        case 3:
            // Get commands from keyboard and mouse
            return COMMANDS_KEYBOARD_MOUSE;
        
        default:
            return state_remote_commands;
    }
}

uint8_t _state_machine_chassis() {
    
    switch (state_remote_commands) {
        
        case COMMANDS_REMOTE_CONTROLLER:
            // Remote controller
            return _state_machine_chassis_remote_controller();
        
        case COMMANDS_KEYBOARD_MOUSE:
            // Keyboard and mouse
            return _state_machine_chassis_keyboard_mouse();
        
        default:
            return state_chassis;
    }
}

uint8_t _state_machine_chassis_remote_controller() {
    
    switch(remote_ctrl.rc.s[1]) {
        
        case 1:
            // Chassis-follow-gimbal
            return CHASSIS_FOLLOW_GIMBAL;
        
        case 3:
            // Contiguous rotation
            return CHASSIS_CONTIGUOUS_ROTATION;
				
				case 2:
            // Chassis-follow-gimbal
            return CHASSIS_FOLLOW_GIMBAL;
        
        default:
            return state_chassis;
    }
}

uint8_t _state_machine_chassis_keyboard_mouse() {
    
    if (KeyBoard_SHIFT) {
        // Contiguous rotation
		return CHASSIS_CONTIGUOUS_ROTATION;
        
    }
    
    // Chassis-follow-gimbal
	return CHASSIS_FOLLOW_GIMBAL;
}

uint8_t _state_machine_gimbal() {
    
    switch (state_remote_commands) {
        
        case COMMANDS_REMOTE_CONTROLLER:
            // Remote controller
            return _state_machine_gimbal_remote_controller();
        
        case COMMANDS_KEYBOARD_MOUSE:
            // Keyboard and mouse
            return _state_machine_gimbal_keyboard_mouse();
        
        default:
            return state_chassis;
    }
}

uint8_t _state_machine_gimbal_remote_controller() {
    
    // Activate manual aim
    return GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_gimbal_keyboard_mouse() {
    
    if (MOUSE_PRESSED_RIGHT) {
        // Activate auto-aim driven by CV
        return GIMBAL_AUTO_AIM;
    }
    // Activate manual aim
    return GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_shoot_wheels() {
    
    switch (state_remote_commands) {
        
        case COMMANDS_REMOTE_CONTROLLER:
            // Remote controller
            return _state_machine_shoot_wheels_remote_controller();
        
        case COMMANDS_KEYBOARD_MOUSE:
            // Keyboard and mouse
            return _state_machine_shoot_wheels_keyboard_mouse();
        
        default:
            return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_remote_controller() {
    
    if (abs(remote_ctrl.rc.ch[4]) >= shoot_wheels_spin.threshold_rc_wheel_released) {
        // Activate/Continue shooting
        shoot_wheels_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3;
        return SHOOT_WHEELS_SPIN;
    }
    else if (HAL_GetTick() * 1e-3 - shoot_wheels_spin.timestamp_last_shoot_command >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_shoot_wheels) {
        // Deactivate shooting
        return SHOOT_WHEELS_STOP;
    }
    else {
        return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_keyboard_mouse() {
    
    if (MOUSE_PRESSED_LEFT || MOUSE_PRESSED_RIGHT) {
        // Activate/Continue shooting
        shoot_wheels_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3;
        return SHOOT_WHEELS_SPIN;
    }
    else if (HAL_GetTick() * 1e-3 - shoot_wheels_spin.timestamp_last_shoot_command >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_shoot_wheels) {
        // Deactivate shooting
        return SHOOT_WHEELS_STOP;
    }
    else {
        return state_shoot_wheels;
    }
}

uint8_t _state_machine_rev() {
    
    switch (state_remote_commands) {
        
        case COMMANDS_REMOTE_CONTROLLER:
            // Remote controller
            return _state_machine_rev_remote_controller();
        
        case COMMANDS_KEYBOARD_MOUSE:
            // Keyboard and mouse
            return _state_machine_rev_keyboard_mouse();
        
        default:
            return state_rev;
    }
}

uint8_t _state_machine_rev_remote_controller() {
    
    // Manage REV_UNSTUCK state
    if (abs(shooting_motor[2].Data.Current) < 6000)
        time_rev_locked = HAL_GetTick();

    if (HAL_GetTick() - time_rev_locked > 0.3*1e3)
        return REV_UNSTUCK;
    
    if (remote_ctrl.rc.ch[4] >= 300) {
        // Multiple shooting
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3;
        return REV_MULTIPLE_SHOOTING;
    }
    else if (remote_ctrl.rc.ch[4] <= -300) {
        // Single shooting
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3;
        return REV_SINGLE_SHOOTING;
    }
    else {
        // Stop shooting
        return REV_STOP;
    }
}

uint8_t _state_machine_rev_keyboard_mouse() {
    
    // TODO manage REV_UNSTUCK state
    
    if (MOUSE_PRESSED_LEFT) {
        // Activate shooting
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3;
        return state_rev;
    }
    else if ((MOUSE_PRESSED_LEFT) && ((HAL_GetTick() * 1e-3 - rev_spin.timestamp_last_shoot_command) < rev_spin.time_threshold_hold_mouse_key_multiple_shooting)) {
        // Triple shooting
		
        return REV_TRIPLE_SHOOTING;

    }
    else if (MOUSE_PRESSED_LEFT && HAL_GetTick() * 1e-3 - rev_spin.timestamp_last_shoot_command >= rev_spin.time_threshold_hold_mouse_key_multiple_shooting) {
        // Multiple shooting
        return REV_MULTIPLE_SHOOTING;
    }
    else {
        // Stop shooting
        return REV_STOP;
    }
}