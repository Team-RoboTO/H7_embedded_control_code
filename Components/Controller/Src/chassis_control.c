/**
 * @file    chassis_control.c
 * @brief   Chassis control loop � reference generation for the 4-wheel drive.
 *
 * @details
 * This module computes per-wheel angular velocity references from operator or
 * autonomous inputs and sends them to the MIT-mode motors each control tick
 * via control_loop_chassis().
 *
 * +---------------------------------------------------------------------------+
 * � Chassis state            � Behaviour                                      �
 * +--------------------------+------------------------------------------------�
 * � FOLLOW_GIMBAL            � Chassis aligns to gimbal yaw via a proportional�
 * �                          � correction on the yaw error. While is_rotating �
 * �                          � is set, field-oriented decomposition is applied �
 * �                          � until the head re-aligns, then it clears.      �
 * � CONTIGUOUS_ROTATION      � Full field-oriented movement with constant max  �
 * �                          � yaw spin (chassis spins continuously). Sets    �
 * �                          � is_rotating = 1 for the return-to-align logic. �
 * +---------------------------------------------------------------------------+
 *
 * Command sources (selected by state_remote_commands):
 *   - REMOTE_CONTROLLER : raw RC channels [-660, +660]
 *   - KEYBOARD_MOUSE    : WASD keys (logic commented out, pending)
 *   - AUTONOMUS         : velocity targets from CV pipeline (fwd_bwd_cv,
 *                         left_right_cv), scaled by wheel radius
 *
 * Wheel mixing (mecanum kinematics):
 *   r_x[0] = (+bwd_fwd - left_right) + yaw     wheel 1
 *   r_x[1] = (+bwd_fwd + left_right) + yaw     wheel 2
 *   r_x[2] = (-bwd_fwd + left_right) + yaw     wheel 3
 *   r_x[3] = (-bwd_fwd - left_right) + yaw     wheel 4
 */

#include "chassis_control.h"
#include "robot_config.h"
#include "state_machine.h"
#include "PID.h"
#include "Remote_Control.h"
#include "Cubemars_Motor.h"
#include "Damiao_Motor.h"
#include "power_estimation.h"
#include "math_utils.h"
#include "control_utils.h"
#include "MiniPC.h"

#include "rtt_log.h"
#include "segger_rtt.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_MIT_t chassis = {
    
    .n          = 4,  // Number of system states
    .m          = 4,  // Number of system inputs
    .p          = 5,  // Number of system outputs
    .x          = {0},
    .x_prev     = {0},
    .r_x        = {0},
    .r_x_prev   = {0},
};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

static float max_r_ang_vel_wheels = 45.0;  // Max reference of angular velocity of wheels [rad/s]
uint8_t is_rotating = 0; //flag for complete the rotation until the head return alligned to the zero 

int16_t remote_commands_bwd_fwd;                // in range [-660, +660]
int16_t remote_commands_left_right;             // in range [-660, +660]
static float remote_commands_bwd_fwd_float;     // in range [-660, +660], but float
static float remote_commands_left_right_float;  // in range [-660, +660], but float
static float r_ang_vel_wheel_1_bwd_fwd;         // [rad/s]
static float r_ang_vel_wheel_1_left_right;      // [rad/s]
static float r_ang_vel_wheel_2_bwd_fwd;         // [rad/s]
static float r_ang_vel_wheel_2_left_right;      // [rad/s]
static float r_ang_vel_wheel_3_bwd_fwd;         // [rad/s]
static float r_ang_vel_wheel_3_left_right;      // [rad/s]
static float r_ang_vel_wheel_4_bwd_fwd;         // [rad/s]
static float r_ang_vel_wheel_4_left_right;      // [rad/s]
static float r_ang_vel_wheels_chassis_yaw;      // [rad/s]

static float max_reference = 0; 

uint16_t chassis_power_limit_local = 60;

//MIT variables
static float MIT_p_des = 0.0f; 	    // range -12.5 - +12.5 [rad]
static float MIT_kp    = 0.0f;    	// range 0-500
float MIT_kd           = 0.2f;    	// range 0-5
static float MIT_t_ff  = 0.0f;      // range -15.0 - 15.0 [Nm]

static float radius_wheels = 0.0825f;

// Acceleration limiting
float max_acceleration = 40.0f;  // [rad/s�] � tune this value
static float dt_chassis = 0.001f; 

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_chassis() {   
	
    // Update outputs from sensor data
    for (uint8_t i = 0; i < chassis.p; i++) {
        chassis.x_prev[i] = chassis.x[i];
    }
  	chassis.x[4] = nearest_target_angle_from_start_angle(DM_Yaw_Motor.Data.Position - GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD, 0);
    
    // Remote commands
		switch (state_remote_commands) {
        
            case COMMANDS_REMOTE_CONTROLLER:
                // Update commands from remote controller
                remote_commands_bwd_fwd     = RC_info.RC.Channel[2];
                remote_commands_left_right  = RC_info.RC.Channel[3];
                break;
						
            case COMMANDS_AUTONOMUS:
							 // Update commands from CV
							 remote_commands_bwd_fwd = fwd_bwd_cv*sqrt(2)*radius_wheels;
							 remote_commands_left_right = left_right_cv*sqrt(2)*radius_wheels;
					 
						case COMMANDS_KEYBOARD_MOUSE:
//                // Update commands from keyboard
////                compute_weights_WASD_keys(dt_chassis);
////                remote_commands_bwd_fwd_float     = MAX_RC_TILT * weight_fwd_key;
////                remote_commands_bwd_fwd_float    -= MAX_RC_TILT * weight_bwd_key;
////                remote_commands_left_right_float  = MAX_RC_TILT * weight_right_key;
////                remote_commands_left_right_float -= MAX_RC_TILT * weight_left_key;
//                saturate(&remote_commands_bwd_fwd_float,    MAX_RC_TILT);  // Value should never reach saturation, but keep it for safety
//                saturate(&remote_commands_left_right_float, MAX_RC_TILT);  // Value should never reach saturation, but keep it for safety
//                // Convert from float to int16
//                remote_commands_bwd_fwd    = -(int16_t) remote_commands_bwd_fwd_float;
//                remote_commands_left_right = -(int16_t) remote_commands_left_right_float;
//                break;

            default:
                remote_commands_bwd_fwd     = 0;
                remote_commands_left_right  = 0;
                break;
    }
    // Update references
    for (uint8_t i = 0; i < chassis.n; i++) {
        chassis.r_x_prev[i] = chassis.r_x[i];
    }
		
    switch (state_chassis) {
                
        case CHASSIS_FOLLOW_GIMBAL:
				
					if (is_rotating == 0) {
						// Forward/Backward
						r_ang_vel_wheel_1_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_2_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_3_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_4_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels;
						// Left/Right
						r_ang_vel_wheel_1_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_2_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_3_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels;
						r_ang_vel_wheel_4_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels;
						// Align chassis to gimbal
						r_ang_vel_wheels_chassis_yaw    = max(min(chassis.x[4], pi/2), -pi/2) * (2/pi) * max_r_ang_vel_wheels;
						break;
					}
					
					else if (is_rotating == 1 && ((DM_Yaw_Motor.Data.Angle_sum > (GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD + 1000)) || (DM_Yaw_Motor.Data.Angle_sum < (GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD)))) {
					 // Forward/Backward
            r_ang_vel_wheel_1_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_2_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_3_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_4_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(- chassis.x[4] + pi/4);
            // Left/Right
            r_ang_vel_wheel_1_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_2_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_3_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_4_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(+ chassis.x[4] + pi/4);
            // Chassis contiguous rotation
            r_ang_vel_wheels_chassis_yaw    = 20;//max_r_ang_vel_wheels;
						break;
					}
					
					else {
						is_rotating = 0;
						break;
					}
        
        case CHASSIS_CONTIGUOUS_ROTATION:
            // Forward/Backward
            r_ang_vel_wheel_1_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_2_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_3_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(- chassis.x[4] + pi/4);
            r_ang_vel_wheel_4_bwd_fwd       = ((float) remote_commands_bwd_fwd / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(- chassis.x[4] + pi/4);
            // Left/Right
            r_ang_vel_wheel_1_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_2_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_3_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * cos(+ chassis.x[4] + pi/4);
            r_ang_vel_wheel_4_left_right    = ((float) remote_commands_left_right / MAX_RC_TILT) * max_r_ang_vel_wheels * sin(+ chassis.x[4] + pi/4);
            // Chassis contiguous rotation
            r_ang_vel_wheels_chassis_yaw    = 30;//max_r_ang_vel_wheels;
						is_rotating = 1;
            break;
  
        default:
            break;
    }
    
		//generation of the referce to send to the MIT motor
    chassis.r_x[0] = (+ r_ang_vel_wheel_1_bwd_fwd - r_ang_vel_wheel_1_left_right) + r_ang_vel_wheels_chassis_yaw;
		chassis.r_x[1] = (+ r_ang_vel_wheel_2_bwd_fwd + r_ang_vel_wheel_2_left_right) + r_ang_vel_wheels_chassis_yaw;
		chassis.r_x[2] = (- r_ang_vel_wheel_3_bwd_fwd + r_ang_vel_wheel_3_left_right) + r_ang_vel_wheels_chassis_yaw;
		chassis.r_x[3] = (- r_ang_vel_wheel_4_bwd_fwd - r_ang_vel_wheel_4_left_right) + r_ang_vel_wheels_chassis_yaw;
		
		/*
	  * Acceleration limiter: clamp the per-tick reference delta so that no wheel
	  * is commanded to change speed faster than max_acceleration [rad/s�].
	  * All four wheels are clamped independently; the saturation block that
	  * follows will re-scale proportionally if any wheel still exceeds the
	  * physical limit after ramping.
	  */
		if (is_rotating == 0){
			float max_delta = max_acceleration * dt_chassis;
			for (uint8_t i = 0; i < chassis.n; i++) {
				float delta = chassis.r_x[i] - chassis.r_x_prev[i];
				
			if (fabsf(chassis.r_x[i]) > fabsf(chassis.r_x_prev[i])) {
					if (delta >  max_delta) delta =  max_delta;
					if (delta < -max_delta) delta = -max_delta;
					chassis.r_x[i] = chassis.r_x_prev[i] + delta;
					}
			}
		}

		/*
		Optimal saturation: if one wheel attempts to reach a reference value higher than the
		physical limit of the wheel, all wheel references are scaled down by the same factor.
		Otherwise, the robot would not follow the correct movement because the wheels would
		reach velocities with proportions different from those imposed by the kinematic model,
		resulting in an incorrect motion profile.
		*/
		max_reference = 0; //max speed reference
		for (uint8_t i = 0; i < chassis.n; i++) {
			float abs_val = fabsf(chassis.r_x[i]);
			if (abs_val > max_reference) max_reference = abs_val;
    }
		if (max_reference > (float)(max_r_ang_vel_wheels)) {
			for (uint8_t i = 0; i < chassis.n; i++) chassis.r_x[i] *= (44/max_reference);
		}

			float data[] = { vx_ref, vx, 0 };
			RTT_Log(data, 3);
		
		// Competition Power Limit
		#if IS_POWER_LIMIT_ENABLED
			chassis_power_limit_local	= robot_status.chassis_power_limit-20;
		
			chassis_power_control( chassis_power_limit_local , chassis.u);
		#endif
}

