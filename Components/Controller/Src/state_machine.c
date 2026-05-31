#include "state_machine.h"

#include "Remote_Control.h"
#include "shooting_control.h"
#include "stm32h7xx_hal.h"
#include "math_utils.h"
#include <stdlib.h>
#include "DJI_Motor.h"
#include "Damiao_Motor.h"
#include "Minipc.h"

  /**************/
 /*   STATES   */
/**************/

uint8_t state_remote_commands = COMMANDS_STOP;
uint8_t state_chassis         = CHASSIS_FOLLOW_GIMBAL;
uint8_t state_gimbal          = GIMBAL_MANUAL_AIM;
uint8_t state_shoot_wheels    = SHOOT_WHEELS_STOP;
uint8_t state_rev             = REV_STOP;
uint8_t state_push            = PUSH_STOP;
uint8_t state_lidar_lifter    = LIDAR_DOWN;  // FIX: was bare `DOWN` (undefined symbol)

extern int g_spinspin_mode;
static uint16_t last_shift_state = 0;  // file-local; no need to be global

  /********************************/
 /*   SHOOT WHEELS SPIN CONFIG   */
/********************************/

#if IS_MATCH_MODE_ENABLED
    #define TIME_SHOOTING_WHEELS  60.0f  // [s]
#else
    #define TIME_SHOOTING_WHEELS   3.0f  // [s]
#endif

shoot_wheels_spin_t shoot_wheels_spin = {
    .threshold_rc_wheel_released                              = 100,
    .timestamp_last_shoot_command                             = 0.0f,
    .time_without_shoot_commands_before_stopping_wheels       = TIME_SHOOTING_WHEELS
};

  /***************************/
 /*   REV / PUSH CONFIG     */
/***************************/

rev_spin_t rev_spin = {
    .timestamp_last_shoot_command                    = 0.0f,
    .time_threshold_hold_mouse_key_multiple_shooting = 0.2f,
    .time_rev_locked                                 = 0.0f
};

push_spin_t push_spin = {
    .timestamp_last_shoot_command                    = 0.0f,
    .time_threshold_hold_mouse_key_multiple_shooting = 0.2f,
    .time_rev_locked                                 = 0.0f
};

  /******************************/
 /*   BARREL HEAT MANAGEMENT   */
/******************************/

barrel_heat_management_t barrel_heat = {
    .heat_limit            = 80.0f,
    .current_heat          = 0.0f,
    .cooling_rate          = 12.0f,
    .heat_per_projectile   = 10.0f,
    .safe_threshold        = 0,
    .last_shooting_position = 0.0f,
    .last_cool_time        = 0.0f
};

/**
 * Track barrel heat locally between Referee System updates.
 *
 * Called once per state-machine tick (STD / SENTRY path only).
 * Adds heat when the revolver has advanced by ~π/4 rad since the last
 * counted shot, and drains heat at cooling_rate every 100 ms.
 *
 * NOTE: This is a local estimator. Override current_heat and heat_limit
 * with Referee System values whenever a fresh packet arrives so that the
 * estimator stays anchored to ground truth.
 */
void _update_barrel_heat_logic(void) {

    // --- Shot detection via revolver encoder ---
    float angle_delta = (float)DJI_Rev_Motor.Data.Angle_sum
                        - barrel_heat.last_shooting_position;

    if (angle_delta > (float)pi / 4.0f - 0.01f) {
        barrel_heat.last_shooting_position = (float)DJI_Rev_Motor.Data.Angle_sum;
        barrel_heat.current_heat += barrel_heat.heat_per_projectile;
    }

    // --- Cooling: drain at cooling_rate [units/s], sampled every 100 ms ---
    uint32_t now_ms = HAL_GetTick();
    if (now_ms - (uint32_t)barrel_heat.last_cool_time >= 100) {
        barrel_heat.last_cool_time = (float)now_ms;

        barrel_heat.current_heat -= barrel_heat.cooling_rate / 10.0f;
        if (barrel_heat.current_heat < 0.0f) {
            barrel_heat.current_heat = 0.0f;
        }
    }
}

  /************************/
 /*   TOP-LEVEL UPDATE   */
/************************/

void robot_states_update_state_machine(void) {
    state_remote_commands = _state_machine_remote_commands();
    state_chassis         = _state_machine_chassis();
    state_gimbal          = _state_machine_gimbal();
    state_shoot_wheels    = _state_machine_shoot_wheels();
    state_rev             = _state_machine_rev();
    state_push            = _state_machine_push();
    state_lidar_lifter    = _state_machine_lidar_lifter();
}

  /******************************/
 /*   REMOTE COMMANDS STATES   */
/******************************/

uint8_t _state_machine_remote_commands(void) {

    switch (RC_info.RC.Switch) {
        case 0:  return COMMANDS_STOP;
        case 1:  return (RC_info.RC.Right == 0) ? COMMANDS_REMOTE_CONTROLLER
                                                : COMMANDS_KEYBOARD_MOUSE;
        case 2:  return COMMANDS_AUTONOMUS;
        default: return state_remote_commands;  // hold current state
    }
}

  /***********************/
 /*   CHASSIS STATES    */
/***********************/

uint8_t _state_machine_chassis(void) {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_chassis_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_chassis_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_chassis_autonomus();
        default:                         return state_chassis;
    }
}

uint8_t _state_machine_chassis_remote_controller(void) {
    return (RC_info.RC.Trigger == 1) ? CHASSIS_CONTIGUOUS_ROTATION
                                     : CHASSIS_FOLLOW_GIMBAL;
}

uint8_t _state_machine_chassis_keyboard_mouse(void) {

    // Toggle spin mode on the rising edge of SHIFT
    if (RC_info.Key.Set.SHIFT && !last_shift_state) {
        g_spinspin_mode = !g_spinspin_mode;
    }
    last_shift_state = RC_info.Key.Set.SHIFT;

    return g_spinspin_mode ? CHASSIS_CONTIGUOUS_ROTATION : CHASSIS_FOLLOW_GIMBAL;
}

uint8_t _state_machine_chassis_autonomus(void) {
    // TODO: implement autonomous chassis logic
    return CHASSIS_FOLLOW_GIMBAL;
}

  /*********************/
 /*   GIMBAL STATES   */
/*********************/

uint8_t _state_machine_gimbal(void) {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_gimbal_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_gimbal_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_gimbal_autonomus();
        default:                         return state_gimbal;
    }
}

uint8_t _state_machine_gimbal_remote_controller(void) {
    return GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_gimbal_keyboard_mouse(void) {
    return RC_info.Mouse.Press_R ? GIMBAL_AUTO_AIM : GIMBAL_MANUAL_AIM;
}

uint8_t _state_machine_gimbal_autonomus(void) {
    // TODO: implement autonomous gimbal logic
    return GIMBAL_AUTO_AIM;
}

  /*****************************/
 /*   SHOOT WHEELS STATES     */
/*****************************/

uint8_t _state_machine_shoot_wheels(void) {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_shoot_wheels_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_shoot_wheels_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_shoot_wheels_autonomus();
        default:                         return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_remote_controller(void) {

    float now_s = HAL_GetTick() * 1e-3f;

    if (abs(RC_info.RC.Wheel) >= shoot_wheels_spin.threshold_rc_wheel_released) {
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;  // within spin-down window
}

uint8_t _state_machine_shoot_wheels_keyboard_mouse(void) {

    float now_s = HAL_GetTick() * 1e-3f;

    if (RC_info.Mouse.Press_L || RC_info.Mouse.Press_R) {
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;
}

uint8_t _state_machine_shoot_wheels_autonomus(void) {

    float now_s = HAL_GetTick() * 1e-3f;

    if (shoot_flag_cv) {
        shoot_wheels_spin.timestamp_last_shoot_command = now_s;
        return SHOOT_WHEELS_SPIN;
    }
    if (now_s - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_without_shoot_commands_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;
}

#if IS_STD || IS_SENTRY
/* ------------------------------------------------------------------ */
/*   Shared jam-detection helper                                       */
/*                                                                     */
/*   Monitors DJI_Rev_Motor.Data.Current and advances the stall timer */
/*   when current is high (motor fighting a jam).                      */
/*   Returns REV_UNSTUCK, REV_STOP (after prolonged stall), or        */
/*   REV_STOP (no stall) via *out_state.                               */
/*   Returns 1 if a stall override should take effect, 0 otherwise.   */
/* ------------------------------------------------------------------ */
static int _check_rev_jam(uint8_t *out_state) {

    // Motor drawing low current → not stalled; reset timer
    if (abs(DJI_Rev_Motor.Data.Current) < 6000) {
        rev_spin.time_rev_locked = (float)HAL_GetTick();
        return 0;
    }

    uint32_t stall_ms = HAL_GetTick() - (uint32_t)rev_spin.time_rev_locked;

    if (stall_ms > 1000) {
        // Prolonged stall: give up and reset
        unstuck_rev_enabled = 0;
        rev_spin.time_rev_locked = (float)HAL_GetTick();
        *out_state = REV_STOP;
        return 1;
    }

    if (stall_ms > 300) {
        *out_state = REV_UNSTUCK;
        return 1;
    }

    return 0;
}

  /*******************/
 /*   REV STATES    */
/*******************/

uint8_t _state_machine_rev(void) {
    _update_barrel_heat_logic();

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_rev_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_rev_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_rev_autonomus();
        default:                         return state_rev;
    }
}

uint8_t _state_machine_rev_remote_controller(void) {

    // Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Jam detection
    uint8_t jam_state;
    if (_check_rev_jam(&jam_state)) return jam_state;

    // Normal wheel control
    if (RC_info.RC.Wheel >= 300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_MULTIPLE_SHOOTING;
    }
    if (RC_info.RC.Wheel <= -300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_SINGLE_SHOOTING;
    }
    return REV_STOP;
}

uint8_t _state_machine_rev_keyboard_mouse(void) {

    // Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Jam detection
    uint8_t jam_state;
    if (_check_rev_jam(&jam_state)) return jam_state;

    float now_s       = HAL_GetTick() * 1e-3f;
    float held_time_s = now_s - rev_spin.timestamp_last_shoot_command;

    if (RC_info.Mouse.Press_L) {
        if (rev_spin.timestamp_last_shoot_command == 0.0f) {
            // Rising edge: record press timestamp
            rev_spin.timestamp_last_shoot_command = now_s;
            return REV_SINGLE_SHOOTING;  // FIX: was missing a return for initial press
        }
        if (held_time_s >= rev_spin.time_threshold_hold_mouse_key_multiple_shooting) {
            return REV_MULTIPLE_SHOOTING;
        }
        return REV_SINGLE_SHOOTING;  // FIX: was falling off with no return → UB
    }

    // Button released
    rev_spin.timestamp_last_shoot_command = 0.0f;
    return REV_STOP;
}

uint8_t _state_machine_rev_autonomus(void) {

    // Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Jam detection
    uint8_t jam_state;
    if (_check_rev_jam(&jam_state)) return jam_state;

    float now_s = HAL_GetTick() * 1e-3f;

    // FIX: original checked `timestamp_last_shoot_command > 0.2f` (absolute value, meaningless).
    //      Correct intent: fire when CV requests it AND enough time has elapsed since last shot.
    if (shoot_flag_cv &&
        (now_s - rev_spin.timestamp_last_shoot_command) >= 0.2f) {
        rev_spin.timestamp_last_shoot_command = now_s;
        return REV_SINGLE_SHOOTING;
    }
    return REV_STOP;
}

/* Push state machine is not used on STD/SENTRY; provide a stub to
   satisfy the linker when building a unified binary. */
uint8_t _state_machine_push(void)                    { return PUSH_STOP; }
uint8_t _state_machine_push_remote_controller(void)  { return PUSH_STOP; }
uint8_t _state_machine_push_keyboard_mouse(void)     { return PUSH_STOP; }
uint8_t _state_machine_push_autonomus(void)          { return PUSH_STOP; }

#elif IS_HERO

  /*******************/
 /*   REV STATES    */
/*******************/

uint8_t _state_machine_rev(void) {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_rev_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_rev_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_rev_autonomus();
        default:                         return state_rev;
    }
}

uint8_t _state_machine_rev_remote_controller(void) {

    // Hero fires one projectile per wheel click regardless of direction
    if (RC_info.RC.Wheel >= 300 || RC_info.RC.Wheel <= -300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return REV_SINGLE_SHOOTING;
    }
    return REV_STOP;
}

uint8_t _state_machine_rev_keyboard_mouse(void) {

    // TODO: add jam detection (REV_UNSTUCK) as in remote-controller mode
    float now_s       = HAL_GetTick() * 1e-3f;
    float held_time_s = now_s - rev_spin.timestamp_last_shoot_command;

    if (RC_info.Mouse.Press_L) {
        if (rev_spin.timestamp_last_shoot_command == 0.0f) {
            rev_spin.timestamp_last_shoot_command = now_s;
            return REV_SINGLE_SHOOTING;
        }
        if (held_time_s >= rev_spin.time_threshold_hold_mouse_key_multiple_shooting) {
            return REV_SINGLE_SHOOTING;  // Hero: continuous-fire is still single-shot cadence
        }
        return REV_SINGLE_SHOOTING;
    }

    rev_spin.timestamp_last_shoot_command = 0.0f;
    return REV_STOP;
}

uint8_t _state_machine_rev_autonomus(void) {

    // TODO: add jam detection
    if (HAL_GetTick() - (uint32_t)rev_spin.time_rev_locked > 300)
        return REV_UNSTUCK;

    float now_s = HAL_GetTick() * 1e-3f;

    // FIX: same as STD — check elapsed time, not absolute timestamp
    if (shoot_flag_cv &&
        (now_s - rev_spin.timestamp_last_shoot_command) >= 0.2f) {
        rev_spin.timestamp_last_shoot_command = now_s;
        return REV_SINGLE_SHOOTING;
    }
    return REV_STOP;
}

  /********************/
 /*   PUSH STATES    */
/********************/

uint8_t _state_machine_push(void) {

    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_push_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_push_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_push_autonomus();
        default:                         return state_push;
    }
}

uint8_t _state_machine_push_remote_controller(void) {

    // FIX: both branches now use push_spin (was using rev_spin on the second branch)
    if (RC_info.RC.Wheel >= 300 || RC_info.RC.Wheel <= -300) {
        push_spin.timestamp_last_shoot_command = HAL_GetTick() * 1e-3f;
        return PUSH_SINGLE_SHOOTING;
    }
    return PUSH_STOP;
}

uint8_t _state_machine_push_keyboard_mouse(void) {

    // FIX: all accesses now use push_spin instead of rev_spin
    float now_s       = HAL_GetTick() * 1e-3f;
    float held_time_s = now_s - push_spin.timestamp_last_shoot_command;

    if (RC_info.Mouse.Press_L) {
        if (push_spin.timestamp_last_shoot_command == 0.0f) {
            push_spin.timestamp_last_shoot_command = now_s;
            return PUSH_SINGLE_SHOOTING;
        }
        if (held_time_s >= push_spin.time_threshold_hold_mouse_key_multiple_shooting) {
            return PUSH_SINGLE_SHOOTING;
        }
        return PUSH_SINGLE_SHOOTING;
    }

    push_spin.timestamp_last_shoot_command = 0.0f;
    return PUSH_STOP;
}

uint8_t _state_machine_push_autonomus(void) {

    // FIX: use push_spin.time_rev_locked, not rev_spin
    if (HAL_GetTick() - (uint32_t)push_spin.time_rev_locked > 300)
        return PUSH_UNSTUCK;

    float now_s = HAL_GetTick() * 1e-3f;

    if (shoot_flag_cv &&
        (now_s - push_spin.timestamp_last_shoot_command) >= 0.2f) {
        push_spin.timestamp_last_shoot_command = now_s;
        return PUSH_SINGLE_SHOOTING;
    }
    return PUSH_STOP;
}

#endif /* IS_HERO */

  /*************************/
 /*   LIDAR LIFTER STATE  */
/*************************/

uint8_t _state_machine_lidar_lifter(void) {
    return (RC_info.RC.Stop == 1) ? LIDAR_UP : LIDAR_DOWN;
}