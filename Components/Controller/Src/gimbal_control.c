/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gimbal_control.c
  * @brief          : Control of yaw and pitch motors 
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gimbal_control.h"
#include "Robot_config.h"
#include "state_machine.h"
#include "INS_task.h"
#include "math_utils.h"
#include "Remote_Control.h"
#include "chassis_control.h"
#include "PID.h"
#include "cubemars_motor.h"
#include "damiao_motor.h"
#include "MiniPC.h"
#include "control_utils.h"

#include "rtt_log.h"
#include "segger_rtt.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t gimbal = {
    .n          = 2,
    .m          = 2,
    .p          = 2,
    .x          = {0},
    .x_prev     = {0},
    .u          = {0},
    .u_prev     = {0},
    .ud         = {0},
    .ud_prev    = {0},
    .r_x        = {0},
    .r_x_prev   = {0},
};

  /*******************/
 /*   CONTROLLERS   */
/*******************/

PID_Info_TypeDef pid_yaw_pos;

// Yaw Position PID params: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
float pid_yaw_pos_params[PID_PARAMETER_NUM] = {15.0f, 0.0f, 2.0f, 0.0f, 0.0f, 10.0f, 30.0f};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

static float remote_commands_yaw;
static float remote_commands_pitch;

static float time_stamp_cv = 0;
static float yaw_command_from_cv = 0;
static float pitch_command_from_cv = 0;
static float time_stamp_cv_prev;
static float yaw_command_from_cv_prev;
static float pitch_command_from_cv_prev;

static float k_ff_yaw = 0.3f;

static bool is_first_iter = true;
static bool is_homing = true;

float pitch_zero;

float lim_ang_pitch = 0.02f;
float lim_ang_yaw = 0.03f;

static LowPassFilter1p_Info_TypeDef lpf_pitch_cv;
#define CV_PITCH_LPF_ALPHA 0.65f

static LowPassFilter1p_Info_TypeDef lpf_yaw_cv;
#define CV_YAW_LPF_ALPHA 0.8f

float pitch_cv_filtered = 0;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_gimbal() {
    // update system state from IMU/INS sensors
    for (uint8_t i = 0; i < gimbal.p; i++) {
        gimbal.x_prev[i] = gimbal.x[i];
    }
    gimbal.x[0] = INS_Info.Yaw_TolAngle * DEG_TO_RAD;     // yaw position  [rad]
    gimbal.x[1] = INS_Info.Roll_Angle * DEG_TO_RAD;   // pitch position [rad] (IMU is likely rotated 90deg)

    // update reference history
    for (uint8_t i = 0; i < 2; i++) {
        gimbal.r_x_prev[i] = gimbal.r_x[i];
    }
		
    // one-time initialization
    if (is_first_iter) {
        gimbal.r_x[0] = gimbal.x[0];
			  gimbal.r_x[1] = CM_Pitch_Motor.Data.Position;
			
				// PID_INIT
				PID_Init(&pid_yaw_pos, PID_POSITION, pid_yaw_pos_params);
				LowPassFilter1p_Init(&lpf_pitch_cv, CV_PITCH_LPF_ALPHA);
				LowPassFilter1p_Init(&lpf_yaw_cv, CV_YAW_LPF_ALPHA);
			
        is_first_iter = false;
    }
		
		// STOP command: zero all outputs and reset
    if (state_remote_commands == COMMANDS_STOP) {
				
				// reset both PIDs on stop
				pid_yaw_pos.PID_Calc_Clear(&pid_yaw_pos);
			  gimbal.r_x[1] = CM_Pitch_Motor.Data.Position;
			  is_homing = 1;
			  is_first_iter = 1;
        return;
    }
		
		if (is_homing == 1) {
			gimbal.r_x[1] -= 0.01 * 	DEG_TO_RAD;
		  if (gimbal.x[1] >= 0) {
				pitch_zero = gimbal.r_x[1];
				is_homing = 0;
			}
		}
			

    // setpoint generation: manual or auto-aim
    switch (state_gimbal) {

        case GIMBAL_MANUAL_AIM:
            switch (state_remote_commands) {

                case COMMANDS_REMOTE_CONTROLLER:
                    remote_commands_yaw   = -RC_info.RC.Channel[0];
                    remote_commands_pitch = -RC_info.RC.Channel[1];
                        if (remote_commands_yaw != 0){
													 gimbal.r_x[0] += (remote_commands_yaw / MAX_RC_TILT) * 0.15f * DEG_TO_RAD;
												}
												if (is_homing == 0){
													gimbal.r_x[1] = pitch_zero + (remote_commands_pitch / MAX_RC_TILT) * 20 * DEG_TO_RAD;
													saturate_in_range(&gimbal.r_x[1], pitch_zero - 15*DEG_TO_RAD ,pitch_zero + 25*DEG_TO_RAD );
												}
                    break;

                case COMMANDS_KEYBOARD_MOUSE:						
                     remote_commands_yaw   = RC_info.Mouse.X;
                    remote_commands_pitch = RC_info.Mouse.Y;
                    
                    // Mouse movement should be accumulated (delta)
                    // Mouse movement (sensitivity drastically reduced for 1ms loop)
                    gimbal.r_x[0] += (remote_commands_yaw * 0.00001f);
                    gimbal.r_x[1] -= (remote_commands_pitch * 0.00001f); // Inverted Y for standard mouse feel
                    
                    saturate_in_range(&gimbal.r_x[1], pitch_zero - 15*DEG_TO_RAD ,pitch_zero + 25*DEG_TO_RAD );
								break;

                default:
                    break;
            }
            break;

        case GIMBAL_AUTO_AIM:
						yaw_command_from_cv_prev   = yaw_command_from_cv;
						pitch_command_from_cv_prev = pitch_command_from_cv;
						time_stamp_cv_prev         = time_stamp_cv;

						yaw_command_from_cv   = -yaw_cv;
						pitch_command_from_cv = pitch_cv;
						time_stamp_cv         = time_cv;
				     
			      if (pitch_command_from_cv == 0) pitch_command_from_cv = pitch_command_from_cv_prev;
						if (yaw_command_from_cv == 0) yaw_command_from_cv = yaw_command_from_cv_prev;
						
						if (time_stamp_cv != time_stamp_cv_prev) {
							
							//gimbal.r_x[0] = yaw_command_from_cv;
//							gimbal.r_x[1] = -pitch_command_from_cv;
//						// dt between last two CV frames (seconds)
//						  float cv_dt = time_stamp_cv - time_stamp_cv_prev;
//							float yaw_target_vel   = (yaw_command_from_cv - yaw_command_from_cv_prev) / cv_dt;
//							float pitch_target_vel = (pitch_command_from_cv - pitch_command_from_cv_prev) / cv_dt;

//							// Lead the reference by one frame (feedforward)
//							gimbal.r_x[0] += yaw_target_vel   * cv_dt;
//							gimbal.r_x[1] += pitch_target_vel * cv_dt;
						}
													// Standard delta: New Target - Previous Value
							float limit_pitch = lim_ang_pitch * DEG_TO_RAD;
							float delta_pitch = pitch_command_from_cv - pitch_command_from_cv_prev; 
						
							float limit_yaw = lim_ang_yaw * DEG_TO_RAD;
						  float delta_yaw = yaw_command_from_cv - yaw_command_from_cv_prev; 
						  
							// Clamp the command pitch
								if (delta_pitch >= limit_pitch) {
										pitch_command_from_cv = pitch_command_from_cv_prev + limit_pitch;
										gimbal.r_x[1] = -pitch_command_from_cv;
								} else if (delta_pitch < -limit_pitch) {
										pitch_command_from_cv = pitch_command_from_cv_prev - limit_pitch;
										gimbal.r_x[1] = -pitch_command_from_cv;
								} 
							
							// Clamp the command yaw
							if (delta_yaw >= limit_yaw) {
									yaw_command_from_cv = yaw_command_from_cv_prev + limit_yaw;
									gimbal.r_x[0] = -yaw_command_from_cv;
							} else if (delta_yaw < -limit_yaw) {
									yaw_command_from_cv = yaw_command_from_cv_prev - limit_yaw;
									gimbal.r_x[0] = -yaw_command_from_cv;
							}
							
							
							//else gimbal.r_x[1] = -pitch_command_from_cv;
							saturate_in_range(&gimbal.r_x[1], pitch_zero - 5*DEG_TO_RAD ,pitch_zero + 20*DEG_TO_RAD );
							
							gimbal.r_x[0] = LowPassFilter1p_Update(&lpf_yaw_cv, gimbal.r_x[0]);
							gimbal.r_x[1] = LowPassFilter1p_Update(&lpf_pitch_cv, gimbal.r_x[1]);
							
						break;
        default:
            break;
    }

  /*****************************/
 /*   CONTROL LOOP EXECUTION  */
/*****************************/

	float pid_yaw_out = PID_Calculate(&pid_yaw_pos, gimbal.r_x[0], gimbal.x[0]);
	gimbal.u[0] = pid_yaw_out - ( k_ff_yaw * w );

}