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
#include "mouse_keyboard_command.h"

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
static float dt_chassis = 0.001f;

static float max_r_ang_vel_wheels = 20.0;  // Max reference of angular velocity of wheels [rad/s]
static float rot_ang_vel_wheels = 45.0f;
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

static uint8_t is_first_iter = true;

float vx;
float vy;
float w;

static float vx_ref;
static float vy_ref;

float v_x_target;
float v_y_target;

static float linear_vel_lim = 4.9f;
static float radius_wheel = 0.0825f;
static float radius_robot = 0.183f;
static float c = 0.7071067812;

static float max_reference = 0; 

uint16_t chassis_power_limit_local = 60;

//MIT variables
float MIT_kd = 0.2f;    	// range 0-5

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_chassis() {

    // Update outputs from sensor data
    for (uint8_t i = 0; i < chassis.p; i++) {
        chassis.x_prev[i] = chassis.x[i];
    }

    // Update wheel velocities from encoder data [rad/s]
    chassis.x[0] = CM_Chassis_Motor[0].Data.Velocity * radius_wheel;
    chassis.x[1] = CM_Chassis_Motor[1].Data.Velocity * radius_wheel;
    chassis.x[2] = CM_Chassis_Motor[2].Data.Velocity * radius_wheel;
    chassis.x[3] = CM_Chassis_Motor[3].Data.Velocity * radius_wheel;

    chassis.x[4] = nearest_target_angle_from_start_angle(DM_Yaw_Motor.Data.Position - GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD, 0);
		
		//Forward kinematics: wheel angular velocities to chassis linear velocities ---
	  vx = ( chassis.x[0] + chassis.x[1] - chassis.x[2] - chassis.x[3]) / (4.0f * c);
    vy = (-chassis.x[0] + chassis.x[1] + chassis.x[2] - chassis.x[3]) / (4.0f * c);
		w  = ( chassis.x[0] + chassis.x[1] + chassis.x[2] + chassis.x[3]) / (4.0f * radius_robot);
		
    // Remote commands
		switch (state_remote_commands) {
        
            case COMMANDS_REMOTE_CONTROLLER:
                // Update commands from remote controller
								v_x_target = ((float) RC_info.RC.Channel[2]    / MAX_RC_TILT) * linear_vel_lim;
							  v_y_target = ((float) RC_info.RC.Channel[3] / MAX_RC_TILT) * linear_vel_lim;
                break;
						
            case COMMANDS_AUTONOMUS:
							 // Update commands from CV
							 v_x_target = fwd_bwd_cv;
							 v_y_target = left_right_cv;
						   saturate(&v_y_target, linear_vel_lim);
							 saturate(&v_x_target, linear_vel_lim);
							 break;
					 
						case COMMANDS_KEYBOARD_MOUSE:
                // Update commands from keyboard
                compute_weights_WASD_keys(dt_chassis);
                remote_commands_bwd_fwd_float     = MAX_RC_TILT * weight_fwd_key;
                remote_commands_bwd_fwd_float    -= MAX_RC_TILT * weight_bwd_key;
                remote_commands_left_right_float  = MAX_RC_TILT * weight_right_key;
                remote_commands_left_right_float -= MAX_RC_TILT * weight_left_key;
                saturate(&remote_commands_bwd_fwd_float,    MAX_RC_TILT);  // Value should never reach saturation, but keep it for safety
                saturate(&remote_commands_left_right_float, MAX_RC_TILT);  // Value should never reach saturation, but keep it for safety
								v_x_target = ((float) remote_commands_bwd_fwd_float    / MAX_RC_TILT) * linear_vel_lim;
							  v_y_target = ((float) remote_commands_left_right_float / MAX_RC_TILT) * linear_vel_lim;
                break;

            default:
                v_x_target  = 0;
                v_y_target  = 0;
                break;
    }
		
    // Update references
    for (uint8_t i = 0; i < chassis.n; i++) {
        chassis.r_x_prev[i] = chassis.r_x[i];
    }

    //Velocity profiler: rate-limit the target before feeding the PID ---
	  slewRateControl(&vx_ref, v_x_target, 5.0f, 25.0f, dt_chassis);
	  slewRateControl(&vy_ref, v_y_target, 5.0f, 25.0f, dt_chassis);

    switch (state_chassis) {
                
        case CHASSIS_FOLLOW_GIMBAL:

            if (is_rotating == 0) {
                // Forward/Backward
                r_ang_vel_wheel_1_bwd_fwd    = vx_ref * c / radius_wheel;
                r_ang_vel_wheel_2_bwd_fwd    = vx_ref * c / radius_wheel;
                r_ang_vel_wheel_3_bwd_fwd    = vx_ref * c / radius_wheel;
                r_ang_vel_wheel_4_bwd_fwd    = vx_ref * c / radius_wheel;
                // Left/Right
                r_ang_vel_wheel_1_left_right = vy_ref * c / radius_wheel;
                r_ang_vel_wheel_2_left_right = vy_ref * c / radius_wheel;
                r_ang_vel_wheel_3_left_right = vy_ref * c / radius_wheel;
                r_ang_vel_wheel_4_left_right = vy_ref * c / radius_wheel;
                // Align chassis to gimbal
                r_ang_vel_wheels_chassis_yaw = max(min(chassis.x[4], pi/2), -pi/2) * (2.0f/pi) * max_r_ang_vel_wheels;
                break;
            }

            else if (is_rotating == 1 && ((DM_Yaw_Motor.Data.Angle_sum > (GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD + 1000)) ||
                                          (DM_Yaw_Motor.Data.Angle_sum < (GIMBAL_YAW_ENCODER_ANGLE_MECH_ZERO_RAD)))) {
							r_ang_vel_wheel_1_bwd_fwd       = (vx_ref * c / radius_wheel) * cos(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_2_bwd_fwd       = (vx_ref * c / radius_wheel) * sin(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_3_bwd_fwd       = (vx_ref * c / radius_wheel) * cos(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_4_bwd_fwd       = (vx_ref * c / radius_wheel) * sin(- chassis.x[4] + pi/4);
							// Left/Right
							r_ang_vel_wheel_1_left_right    = (vy_ref * c / radius_wheel) * cos(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_2_left_right    = (vy_ref * c / radius_wheel) * sin(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_3_left_right    = (vy_ref * c / radius_wheel) * cos(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_4_left_right    = (vy_ref * c / radius_wheel) * sin(+ chassis.x[4] + pi/4);
							// Chassis contiguous rotation
							r_ang_vel_wheels_chassis_yaw    = rot_ang_vel_wheels;
							is_rotating = 1;
							break;
            }

            else {
                is_rotating = 0;
                break;
            }
        
        case CHASSIS_CONTIGUOUS_ROTATION:
            r_ang_vel_wheel_1_bwd_fwd       = (vx_ref * c / radius_wheel) * cos(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_2_bwd_fwd       = (vx_ref * c / radius_wheel) * sin(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_3_bwd_fwd       = (vx_ref * c / radius_wheel) * cos(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_4_bwd_fwd       = (vx_ref * c / radius_wheel) * sin(- chassis.x[4] + pi/4);
						// Left/Right
						r_ang_vel_wheel_1_left_right    = (vy_ref * c / radius_wheel) * cos(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_2_left_right    = (vy_ref * c / radius_wheel) * sin(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_3_left_right    = (vy_ref * c / radius_wheel) * cos(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_4_left_right    = (vy_ref * c / radius_wheel) * sin(+ chassis.x[4] + pi/4);
						// Chassis contiguous rotation
						r_ang_vel_wheels_chassis_yaw    = rot_ang_vel_wheels;
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

//    /*
//     * Optimal saturation: if one wheel exceeds the physical speed limit, scale
//     * all wheels down by the same factor to preserve the correct motion profile.
//     */
//    max_reference = 0.0f;
//    for (uint8_t i = 0; i < chassis.n; i++) {
//        float abs_val = fabsf(chassis.r_x[i]);
//        if (abs_val > max_reference) max_reference = abs_val;
//    }
//		if (max_reference > (float)(max_r_ang_vel_wheels)) {
//			for (uint8_t i = 0; i < chassis.n; i++) chassis.r_x[i] *= (44/max_reference);
//		}
		
		// Competition Power Limit
		#if IS_POWER_LIMIT_ENABLED
			chassis_power_limit_local	= robot_status.chassis_power_limit-20;
		
			chassis_power_control( chassis_power_limit_local , chassis.u);
		#endif
}