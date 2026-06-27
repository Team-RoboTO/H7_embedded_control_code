#include "state_machine.h"

#include "Remote_Control.h"
#include "DJI_Motor.h"
#include "Damiao_Motor.h"
#include "Minipc.h"
#include "Referee_System.h"

  /**************/
 /*   STATES   */
/**************/

uint8_t state_remote_commands = COMMANDS_STOP;
uint8_t state_chassis         = CHASSIS_FOLLOW_GIMBAL;
uint8_t state_gimbal          = GIMBAL_MANUAL_AIM;
uint8_t state_shoot_wheels    = SHOOT_WHEELS_STOP;
uint8_t state_rev             = REV_STOP;
uint8_t state_push            = PUSH_STOP;
uint8_t state_lidar_lifter    = LIDAR_DOWN;  

static bool g_spinspin_mode;
static uint16_t last_shift_state = 0;  // file-local; no need to be global

static uint32_t now_ms = 0;
static uint32_t time_stuck = 0;

	/********************************/
 /*   SHOOT WHEELS SPIN CONFIG   */
/********************************/

static shoot_wheels_spin_t shoot_wheels_spin = {
    .threshold_rc_wheel_released            = 300,
    .timestamp_last_shoot_command           = 0,
    .time_before_stopping_wheels            = 10000,
};

  /***************************/
 /*   REV / PUSH CONFIG     */
/***************************/
static rev_spin_t rev_spin = {
    .timestamp_last_shoot_command         = 0,
	  .time_rev_locked                      = 0, 
		.time_unstuck                         = 0, 
	  #if IS_STD || IS_SENTRY
	  .shooting_frequency                   = 12,
	  #elif IS_HERO
		.shooting_frequency                   = 0.5f,
	  #endif
	  .stuck_state                          = 0
};

  /******************************/
 /*   BARREL HEAT MANAGEMENT   */
/******************************/

static barrel_heat_management_t barrel_heat = {
    .heat_limit             = 80.0f,
    .current_heat           = 0.0f,
    .cooling_rate           = 12.0f,
		#if IS_STD || IS_SENTRY
    .heat_per_projectile    = 10.0f,
		#elif IS_HERO
	  .heat_per_projectile    = 100.0f,
		#endif
    .safe_threshold         = 5,
    .last_cool_time         = 0.0f
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
	if (Referee_System_Info.robot_status.shooter_barrel_heat_limit > 0) barrel_heat.heat_limit = Referee_System_Info.robot_status.shooter_barrel_heat_limit - barrel_heat.safe_threshold;
	if (Referee_System_Info.robot_status.shooter_barrel_cooling_value > 0) barrel_heat.cooling_rate = Referee_System_Info.robot_status.shooter_barrel_cooling_value;
	if (barrel_heat.heat_limit > 100) barrel_heat.safe_threshold = 10;
	
	// --- Cooling: drain at cooling_rate [units/s], sampled every 100 ms ---
	now_ms = HAL_GetTick();
	if (now_ms - (uint32_t)barrel_heat.last_cool_time >= 100) {
			barrel_heat.last_cool_time = now_ms;

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
	  #if IS_STD || IS_SENTRY
			state_lidar_lifter    = _state_machine_lidar_lifter();
		#elif IS_HERO
			state_push            = _state_machine_push();
		#endif
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
    if (flag_rot == 1) return CHASSIS_CONTIGUOUS_ROTATION;
    else return CHASSIS_FOLLOW_GIMBAL;
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
		if (Referee_System_Info.game_status.game_progress == 4){
				return SHOOT_WHEELS_SPIN;
		}
    switch (state_remote_commands) {
        case COMMANDS_REMOTE_CONTROLLER: return _state_machine_shoot_wheels_remote_controller();
        case COMMANDS_KEYBOARD_MOUSE:    return _state_machine_shoot_wheels_keyboard_mouse();
        case COMMANDS_AUTONOMUS:         return _state_machine_shoot_wheels_autonomus();
        default:                         return state_shoot_wheels;
    }
}

uint8_t _state_machine_shoot_wheels_remote_controller(void) {

    if (abs(RC_info.RC.Wheel) >= shoot_wheels_spin.threshold_rc_wheel_released) {
        shoot_wheels_spin.timestamp_last_shoot_command = HAL_GetTick();
        return SHOOT_WHEELS_SPIN;
    }
    if (HAL_GetTick() - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;  // within spin-down window
}

uint8_t _state_machine_shoot_wheels_keyboard_mouse(void) {

    if (RC_info.Mouse.Press_L || RC_info.Mouse.Press_R) {
        shoot_wheels_spin.timestamp_last_shoot_command = HAL_GetTick();
        return SHOOT_WHEELS_SPIN;
    }
    if (HAL_GetTick() - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;
}

uint8_t _state_machine_shoot_wheels_autonomus(void) {

    if (shoot_flag_cv) {
        shoot_wheels_spin.timestamp_last_shoot_command = HAL_GetTick();
        return SHOOT_WHEELS_SPIN;
    }
    if (HAL_GetTick() - shoot_wheels_spin.timestamp_last_shoot_command
            >= shoot_wheels_spin.time_before_stopping_wheels) {
        return SHOOT_WHEELS_STOP;
    }
    return state_shoot_wheels;
}

#if IS_STD || IS_SENTRY
/* ------------------------------------------------------------------ */
/*   Shared stuck-detection function                                  */
/*                                                                    */
/*   Monitors DJI_Rev_Motor.Data.Current and advances the stall timer */
/*   when current is high (motor fighting a jam).                     */
/*                                                                    */
/*   Returns 1 if an unstuck is required and zero otherwise           */
/* ------------------------------------------------------------------ */
int _check_rev_stuck(void) {
	
    if (abs(DJI_Rev_Motor.Data.Current) < 6000) {
        rev_spin.time_rev_locked = HAL_GetTick();
        return 0; 
    }

    uint32_t time_stuck = HAL_GetTick() - rev_spin.time_rev_locked;
    if (time_stuck > 1000){
			rev_spin.time_rev_locked = HAL_GetTick();
			return 0;
		}
    if (time_stuck > 300) {
				rev_spin.time_unstuck = HAL_GetTick();
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

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

    if (RC_info.RC.Wheel >= 300) {
        if (HAL_GetTick() - rev_spin.timestamp_last_shoot_command >= (1000/rev_spin.shooting_frequency)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
        } else {
            return REV_STOP; 
        }
    }
		
//    if (RC_info.RC.Wheel <= -300) {
//        rev_spin.timestamp_last_shoot_command = HAL_GetTick();
//        return REV_SINGLE_SHOOTING;
//    }
    return REV_STOP;
}

uint8_t _state_machine_rev_keyboard_mouse(void) {
		
		// Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

    if (RC_info.Mouse.Press_L || (RC_info.Mouse.Press_R && shoot_flag_cv)) {
        if (HAL_GetTick() - rev_spin.timestamp_last_shoot_command >= (1000/rev_spin.shooting_frequency)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
        } else {
            return REV_STOP; 
        }
    }
		
    return REV_STOP;
}

uint8_t _state_machine_rev_autonomus(void) {

		// Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

    if (shoot_flag_cv) {
        if (HAL_GetTick() - rev_spin.timestamp_last_shoot_command >= (1000/rev_spin.shooting_frequency)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
        } else {
            return REV_STOP; 
        }
    }
		
    return REV_STOP;
}

  /*************************/
 /*   LIDAR LIFTER STATE  */
/*************************/

uint8_t _state_machine_lidar_lifter(void) {
    return (RC_info.RC.Stop == 1) ? LIDAR_UP : LIDAR_DOWN;
}

#elif IS_HERO
int _check_rev_stuck(void) {
	
    if (fabs(DM_Rev_Motor.Data.Torque) < 4.0f) {
        rev_spin.time_rev_locked = HAL_GetTick();
        return 0; 
    }

    time_stuck = HAL_GetTick() - rev_spin.time_rev_locked;
    if (time_stuck > 1000){
			rev_spin.time_rev_locked = HAL_GetTick();
			return 0;
		}
    if (time_stuck > 300) {
				rev_spin.time_unstuck = HAL_GetTick();
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

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

		if (RC_info.RC.Wheel >= 300) {
        if (HAL_GetTick() - rev_spin.timestamp_last_shoot_command >= (1000/rev_spin.shooting_frequency)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
        } else {
            return REV_STOP; 
        }
    }
		
    if (RC_info.RC.Wheel <= -300) {
        rev_spin.timestamp_last_shoot_command = HAL_GetTick();
        return REV_SINGLE_SHOOTING;
    }
    return REV_STOP;
}

uint8_t _state_machine_rev_keyboard_mouse(void) {
		
		// Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

    if (RC_info.Mouse.Press_L || (RC_info.Mouse.Press_R && shoot_flag_cv)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
    }
		
    return REV_STOP;
}

uint8_t _state_machine_rev_autonomus(void) {

		// Heat guard
    if (barrel_heat.current_heat > barrel_heat.heat_limit - barrel_heat.heat_per_projectile)
        return REV_STOP;

    // Stuck detection
    if (_check_rev_stuck()) return REV_UNSTUCK;

    if (shoot_flag_cv) {
        if (HAL_GetTick() - rev_spin.timestamp_last_shoot_command >= (1000/rev_spin.shooting_frequency)) {
            rev_spin.timestamp_last_shoot_command = HAL_GetTick();
						barrel_heat.current_heat += barrel_heat.heat_per_projectile;
						return REV_SINGLE_SHOOTING;
        } else {
            return REV_STOP; 
        }
    }
		
    return REV_STOP;
}

  /********************/
 /*   PUSH STATES    */
/********************/

uint8_t _state_machine_push(void) {

    switch (state_rev) {	
				case REV_SINGLE_SHOOTING:				 return PUSH_SINGLE_SHOOTING;
				case REV_STOP:									 return PUSH_STOP;
				case REV_UNSTUCK:								 return PUSH_STOP;
        default:                         return PUSH_STOP;
    }
}

#endif 