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
#include "state_machine.h"
#include "Cubemars_Motor.h"
#include "Damiao_Motor.h"
#include "power_estimation.h"
#include "control_utils.h"
#include "MiniPC.h"
#include "mouse_keyboard_command.h"
#include "Referee_System.h"
#include "LPF.h"


  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_MIT_t chassis = {
    .n          = 4,  // Number of system states
    .m          = 4,  // Number of system inputs
    .p          = 5,  // Number of system outputs
};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/
static float dt_chassis = 0.001f;

static float max_r_ang_vel_wheels = 20.0;  // Max reference of angular velocity of wheels [rad/s]

static int16_t remote_commands_bwd_fwd;                // in range [-660, +660]
static int16_t remote_commands_left_right;             // in range [-660, +660]
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
bool is_rotating = 0; //flag for complete the rotation until the head return alligned to the zero 
static bool yaw_out_of_range = 0;

float vx;
float vy;
float w;

static float vx_ref;
static float vy_ref;

static float v_x_target;
static float v_y_target;

static float w_ref;
static float w_target;

static float vx_body;
static float vy_body;


static float linear_vel_lim = 4.9f;


static LowPassFilter1p_Info_TypeDef lpf_vx;
static LowPassFilter1p_Info_TypeDef lpf_vy;


#if IS_STD || IS_SENTRY
	static float radius_wheel = 0.0825f;
	static float radius_robot = 0.183f;
	static float rot_ang_vel_wheels = 45.0f;
#elif IS_HERO
	static float radius_wheel = 0.08f;
	static float radius_robot = 0.2895f;
	static float rot_ang_vel_wheels = 15.0f;
#endif

//MIT variables
float MIT_kd = 0.2f;    	// range 0-5


float chassis_power_limit_local = 75;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_chassis() {
	  // Update outputs from sensor data
    for (uint8_t i = 0; i < chassis.p; i++) {
        chassis.x_prev[i] = chassis.x[i];
    }
		
		    // one-time initialization
    if (is_first_iter) {
				LowPassFilter1p_Init(&lpf_vx, LPF_VEL_ALPHA);
				LowPassFilter1p_Init(&lpf_vy, LPF_VEL_ALPHA);
			
        is_first_iter = false;
    }

    // Update wheel velocities from encoder data [rad/s]
    chassis.x[0] = CM_Chassis_Motor[0].Data.Velocity * radius_wheel;
    chassis.x[1] = CM_Chassis_Motor[1].Data.Velocity * radius_wheel;
    chassis.x[2] = CM_Chassis_Motor[2].Data.Velocity * radius_wheel;
    chassis.x[3] = CM_Chassis_Motor[3].Data.Velocity * radius_wheel;

    chassis.x[4] = nearest_target_angle_from_start_angle(DM_Yaw_Motor.Data.Position, 0);
		
		//Forward kinematics: wheel angular velocities to chassis linear velocities ---
	  // Body-frame FK (what you already have)
		vx_body = ( chassis.x[0] + chassis.x[1] - chassis.x[2] - chassis.x[3]) / (4.0f * Cos45);
		vy_body = (-chassis.x[0] + chassis.x[1] + chassis.x[2] - chassis.x[3]) / (4.0f * Cos45);

		// Rotate to world frame using current yaw
		vx = LowPassFilter1p_Update(&lpf_vx, vx_body * cosf(chassis.x[4]) - vy_body * sinf(chassis.x[4]));
		vy = LowPassFilter1p_Update(&lpf_vy, vx_body * sinf(chassis.x[4]) + vy_body * cosf(chassis.x[4]));
		
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
	  
		if (is_rotating == 0){
				#if IS_HERO
				slewRateControl(&vx_ref, v_x_target, 3.0f, 100.0f, dt_chassis);
				slewRateControl(&vy_ref, v_y_target, 3.0f, 100.0f, dt_chassis);
			  #elif IS_STD || IS_SENTRY
				slewRateControl(&vx_ref, v_x_target, 4.0f, 100.0f, dt_chassis);
				slewRateControl(&vy_ref, v_y_target, 4.0f, 100.0f, dt_chassis);
				#endif
		}
		else {
				slewRateControl(&vx_ref, v_x_target, 10.0f, 100.0f, dt_chassis);
				slewRateControl(&vy_ref, v_y_target, 10.0f, 100.0f, dt_chassis);

		}
						
						
    switch (state_chassis) {
                
        case CHASSIS_FOLLOW_GIMBAL:

						yaw_out_of_range = ((DM_Yaw_Motor.Data.Position > 3.0f) ||(DM_Yaw_Motor.Data.Position < 2.0f));

            if (is_rotating == 0) {
								#if IS_HERO
									linear_vel_lim = 2.0f;
									#endif
                // Forward/Backward
                r_ang_vel_wheel_1_bwd_fwd    = vx_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_2_bwd_fwd    = vx_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_3_bwd_fwd    = vx_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_4_bwd_fwd    = vx_ref * Cos45 / radius_wheel;
                // Left/Right
                r_ang_vel_wheel_1_left_right = vy_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_2_left_right = vy_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_3_left_right = vy_ref * Cos45 / radius_wheel;
                r_ang_vel_wheel_4_left_right = vy_ref * Cos45 / radius_wheel;
                // Align chassis to gimbal
                r_ang_vel_wheels_chassis_yaw = max(min(chassis.x[4], pi/2), -pi/2) * (2.0f/pi) * max_r_ang_vel_wheels;
                break;
            }
						            
						else if (is_rotating == 1 && yaw_out_of_range == 1) {
							r_ang_vel_wheel_1_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * cos(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_2_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * sin(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_3_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * cos(- chassis.x[4] + pi/4);
							r_ang_vel_wheel_4_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * sin(- chassis.x[4] + pi/4);
							// Left/Right
							r_ang_vel_wheel_1_left_right    = (vy_ref * Cos45 / radius_wheel) * cos(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_2_left_right    = (vy_ref * Cos45 / radius_wheel) * sin(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_3_left_right    = (vy_ref * Cos45 / radius_wheel) * cos(+ chassis.x[4] + pi/4);
							r_ang_vel_wheel_4_left_right    = (vy_ref * Cos45 / radius_wheel) * sin(+ chassis.x[4] + pi/4);
							// Chassis contiguous rotation
																		
							#if IS_STD || IS_SENTRY
							if (vx_ref == 0 && vy_ref == 0){
								r_ang_vel_wheels_chassis_yaw = rot_ang_vel_wheels;
							}
							else {
									r_ang_vel_wheels_chassis_yaw = 30.0f;
							}
							#elif IS_HERO
							linear_vel_lim = 3.0f;
							if (vx_ref == 0 && vy_ref == 0){
								r_ang_vel_wheels_chassis_yaw = rot_ang_vel_wheels;
							}
							else {
									r_ang_vel_wheels_chassis_yaw = 10.0f;
							}
							#endif
							is_rotating = 1;
							break;
            }

            else {
                is_rotating = 0;
                break;
            }
        
        case CHASSIS_CONTIGUOUS_ROTATION:
            r_ang_vel_wheel_1_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * cos(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_2_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * sin(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_3_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * cos(- chassis.x[4] + pi/4);
						r_ang_vel_wheel_4_bwd_fwd       = (vx_ref * Cos45 / radius_wheel) * sin(- chassis.x[4] + pi/4);
						// Left/Right
						r_ang_vel_wheel_1_left_right    = (vy_ref * Cos45 / radius_wheel) * cos(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_2_left_right    = (vy_ref * Cos45 / radius_wheel) * sin(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_3_left_right    = (vy_ref * Cos45 / radius_wheel) * cos(+ chassis.x[4] + pi/4);
						r_ang_vel_wheel_4_left_right    = (vy_ref * Cos45 / radius_wheel) * sin(+ chassis.x[4] + pi/4);
						// Chassis contiguous rotation
						#if IS_STD || IS_SENTRY
							if (vx_ref == 0 && vy_ref == 0){
								r_ang_vel_wheels_chassis_yaw = rot_ang_vel_wheels;
							}
							else {
									r_ang_vel_wheels_chassis_yaw = 30.0f;
							}
							#elif IS_HERO
							linear_vel_lim = 4.9f;
							if (vx_ref == 0 && vy_ref == 0){
								r_ang_vel_wheels_chassis_yaw = rot_ang_vel_wheels;
							}
							#endif
						//slewRateControl(&r_ang_vel_wheels_chassis_yaw, w_target, 10.0f, 15.0f, dt_chassis);
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
		saturate(&chassis.r_x[0], 44.9f);
		saturate(&chassis.r_x[1], 44.9f);
		saturate(&chassis.r_x[2], 44.9f);
		saturate(&chassis.r_x[3], 44.9f);
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
		  if (Referee_System_Info.robot_status.chassis_power_limit != 0) chassis_power_limit_local	= Referee_System_Info.robot_status.chassis_power_limit;
			else chassis_power_limit_local = 75;
			chassis_power_control(chassis_power_limit_local , chassis.r_x, &MIT_kd);
		#endif
}