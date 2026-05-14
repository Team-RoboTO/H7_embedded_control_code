#include "state_machine.h"

#include "Remote_Control.h"
#include "stm32h7xx_hal.h"
#include "math_utils.h"
#include <stdlib.h>
#include "DJI_Motor.h"
#include "Minipc.h"
  /**************/
 /*   STATES   */
/**************/

uint8_t state_remote_commands  = COMMANDS_STOP;
uint8_t state_chassis          = CHASSIS_FOLLOW_GIMBAL;
uint8_t state_gimbal           = GIMBAL_MANUAL_AIM;
uint8_t state_shoot_wheels     = SHOOT_WHEELS_STOP;
uint8_t state_rev              = REV_STOP;
uint8_t state_lidar_lifter     = DOWN;

  /********************************/
 /*   SHOOT WHEELS SPIN STRUCT   */
/********************************/

#if IS_MATCH_MODE_ENABLED
    #define TIME_SHOOTING_WHEELS  60.0f  // [s]
#else
    #define TIME_SHOOTING_WHEELS  3.0f   // [s]
#endif

shoot_wheels_spin_t shoot_wheels_spin = {
    .threshold_rc_wheel_released                              = 100,   // in range [0, 660]
    .timestamp_last_shoot_command                             = 0.0f,  // [s]
    .time_without_shoot_commands_before_stopping_shoot_wheels = TIME_SHOOTING_WHEELS
};

  /***********************/
 /*   REV SPIN STRUCT   */
/***********************/

rev_spin_t rev_spin = {
    .timestamp_last_shoot_command                    = 0.0f,  // [s]
    .time_threshold_hold_mouse_key_multiple_shooting = 0.2f   // [s]
};

  /************************/
 /*   STATES FUNCTIONS   */
/************************/

void robot_states_update_state_machine() {
    state_remote_commands = _state_machine_remote_commands();
    state_chassis         = _state_machine_chassis();
    state_gimbal          = _state_machine_gimbal();
    state_shoot_wheels    = _state_machine_shoot_wheels();
    state_rev             = _state_machine_rev();
		state_lidar_lifter 		= _state_machine_lidar_lifter();
}

  /******************************/
 /*   REMOTE COMMANDS STATES   */
/******************************/

uint8_t _state_machine_remote_commands() {

    // Software override: if KBM activity is detected, exit STOP mode automatically
    if (RC_info.Key.V != 0 || RC_info.Mouse.X != 0 || RC_info.Mouse.Y != 0)
    {
        return COMMANDS_KEYBOARD_MOUSE;
    }

    switch (RC_info.RC.Switch) {

        case 0:
            return COMMANDS_STOP;

        case 1:
            if (RC_info.RC.Right == 0) return COMMANDS_REMOTE_CONTROLLER;
            else                       return COMMANDS_KEYBOARD_MOUSE;

        case 2:
            return COMMANDS_AUTONOMUS;

        default:
            return state_remote_commands;  // hold current state
    }
}

  /***********************/
 /*   CHASSIS STATES    */
/***********************/

uint8_t _state_machine_chassis() {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_chassis_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_chassis_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_chassis_autonomus();
        default:                         return state_chassis;  // hold current state
    }
}

uint8_t _state_machine_chassis_remote_controller() {

    if (RC_info.RC.Trigger == 1) return CHASSIS_CONTIGUOUS_ROTATION;
    else                         return CHASSIS_FOLLOW_GIMBAL;
}

uint8_t _state_machine_chassis_keyboard_mouse() {

    if (RC_info.Key.Set.SHIFT) return CHASSIS_CONTIGUOUS_ROTATION;
    else                       return CHASSIS_FOLLOW_GIMBAL;
}

uint8_t _state_machine_chassis_autonomus() {

    // TODO: implement autonomous chassis logic
    return CHASSIS_FOLLOW_GIMBAL;  // safe default
}

  /*********************/
 /*   GIMBAL STATES   */
/*********************/

uint8_t _state_machine_gimbal() {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_gimbal_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_gimbal_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_gimbal_autonomus();
        default:                         return state_gimbal;  // hold current state � was wrongly returning state_chassis
    }
}

uint8_t _state_machine_gimbal_remote_controller() {

    return GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_gimbal_keyboard_mouse() {

    if (RC_info.Mouse.Press_R) return GIMBAL_AUTO_AIM;
    else                       return GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_gimbal_autonomus() {

    // TODO: implement autonomous gimbal logic (e.g. always auto-aim)
    return GIMBAL_AUTO_AIM;  // safe default
}

  /*******************************/
 /*   SHOOTING WHEELS STATES    */
/*******************************/

uint8_t _state_machine_shoot_wheels() {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_shoot_wheels_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_shoot_wheels_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_shoot_wheels_autonomus();
        default:                         return state_shoot_wheels;  // hold current state
    }
}

uint8_t _state_machine_shoot_wheels_remote_controller() {

    float now_s = HAL_GetTick() * 1e-3f;

    if (abs(RC_info.RC.Wheel) >= shoot_wheels_spin.threshold_rc_wheel_released) {
        // Wheel active � keep/start spinning
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    else if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
             >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_shoot_wheels) {
        // Timeout expired � stop wheels
        return SHOOT_WHEELS_STOP;
    }
    else {
        // Within timeout window � keep current state (spin-down delay)
        return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_keyboard_mouse() {

    float now_s = HAL_GetTick() * 1e-3f;

    if (RC_info.Mouse.Press_L || RC_info.Mouse.Press_R) {
        // Any mouse button pressed � keep/start spinning
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    else if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
             >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_shoot_wheels) {
        // Timeout expired � stop wheels
        return SHOOT_WHEELS_STOP;
    }
    else {
        // Within timeout window � keep current state (spin-down delay)
        return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_autonomus() {

    float now_s = HAL_GetTick() * 1e-3f;

    if (shoot_flag_cv) {
        // Wheel active � keep/start spinning
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    else if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
             >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_shoot_wheels) {
        // Timeout expired � stop wheels
        return SHOOT_WHEELS_STOP;
    }
    else {
        // Within timeout window � keep current state (spin-down delay)
        return state_shoot_wheels;
    }
}

  /*******************/
 /*   REV STATES    */
/*******************/

uint8_t _state_machine_rev() {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_rev_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_rev_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_rev_autonomus();
        default:                         return state_rev;  // hold current state
    }
}

uint8_t _state_machine_rev_remote_controller() {

    // --- Jam detection: if motor is stalled for too long, trigger unstuck ---
    if (abs(DJI_Rev_Motor.Data.Current) < 6000)
        rev_spin.time_rev_locked = HAL_GetTick();  // reset stall timer (raw ms ticks)

    if (HAL_GetTick() - rev_spin.time_rev_locked > 300)  // 300 ms stall threshold
        return REV_UNSTUCK;

    // --- Normal rev control via wheel axis ---
    if (RC_info.RC.Wheel >= 300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_MULTIPLE_SHOOTING;
    }
    else if (RC_info.RC.Wheel <= -300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_SINGLE_SHOOTING;
    }
    else {
        return REV_STOP;
    }
}

uint8_t _state_machine_rev_keyboard_mouse() {

    // TODO: add jam detection (REV_UNSTUCK) like in remote controller mode

    float now_s        = HAL_GetTick() * 1e-3f;
    float held_time_s  = now_s - rev_spin.timestamp_last_shoot_command;

    if (RC_info.Mouse.Press_L) {

        if (rev_spin.timestamp_last_shoot_command == 0.0f) {
            // Rising edge: button just pressed � record timestamp
            rev_spin.timestamp_last_shoot_command = now_s;
        }

        // Decide based on how long the button has been held
        if (held_time_s >= rev_spin.time_threshold_hold_mouse_key_multiple_shooting) {
            return REV_MULTIPLE_SHOOTING;  // held long enough ? continuous fire
        }
    }
    else {
        // Button released � reset timestamp for next press
        rev_spin.timestamp_last_shoot_command = 0.0f;
        return REV_STOP;
    }
}

uint8_t _state_machine_rev_autonomus() {

    // --- Jam detection: if motor is stalled for too long, trigger unstuck ---
    if (abs(DJI_Rev_Motor.Data.Current) < 6000)
        rev_spin.time_rev_locked = HAL_GetTick();  // reset stall timer (raw ms ticks)

    if (HAL_GetTick() - rev_spin.time_rev_locked > 300)  // 300 ms stall threshold
        return REV_UNSTUCK;

    // --- Normal rev control via wheel axis ---
    else if (shoot_flag_cv && rev_spin.timestamp_last_shoot_command > 0.2f) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_SINGLE_SHOOTING;
    }
    else {
        return REV_STOP;
    }
}

uint8_t _state_machine_lidar_lifter() {
		if( RC_info.RC.Stop == 1) return LIDAR_UP;
		else return LIDAR_DOWN;
}