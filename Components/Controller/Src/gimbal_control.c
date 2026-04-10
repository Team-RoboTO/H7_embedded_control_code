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

static uint8_t is_first_iter = true;

float cm_p_des_origin = -18*DEG_TO_RAD;

float scale_yaw = 0.02f;
float scale_pitch = 0.04f;

static LowPassFilter1p_Info_TypeDef lpf_pitch_cv;
#define CV_PITCH_LPF_ALPHA 0.7f

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
    gimbal.x[1] = INS_Info.Roll_Angle * DEG_TO_RAD;   // pitch position [rad]

    // update reference history
    for (uint8_t i = 0; i < 2; i++) {
        gimbal.r_x_prev[i] = gimbal.r_x[i];
    }
		
    // one-time initialization
    if (is_first_iter) {
        gimbal.r_x[0] = gimbal.x[0];
			
				// PID_INIT
				PID_Init(&pid_yaw_pos, PID_POSITION, pid_yaw_pos_params);
				LowPassFilter1p_Init(&lpf_pitch_cv, CV_PITCH_LPF_ALPHA);
			
        is_first_iter = false;
    }
		
		// STOP command: zero all outputs and reset
    if (state_remote_commands == COMMANDS_STOP) {
				
				// reset both PIDs on stop
				pid_yaw_pos.PID_Calc_Clear(&pid_yaw_pos);
        return;
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
                        gimbal.r_x[1] = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 20 * DEG_TO_RAD;
                    break;

                case COMMANDS_KEYBOARD_MOUSE:						
                     remote_commands_yaw   = RC_info.Mouse.X*0.01;
                    remote_commands_pitch = RC_info.Mouse.Y*0.01;
                    gimbal.r_x[0] += (remote_commands_yaw   / MAX_RC_TILT) * 15 * DEG_TO_RAD;
                    gimbal.r_x[1]  = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 30 * DEG_TO_RAD;              
                    break;

                default:
                    break;
            }
            break;

        case GIMBAL_AUTO_AIM:
						yaw_command_from_cv_prev   = yaw_command_from_cv;
						pitch_command_from_cv_prev = pitch_command_from_cv;
						time_stamp_cv_prev         = time_stamp_cv;

						yaw_command_from_cv   = yaw_cv*scale_yaw;
						pitch_command_from_cv = -pitch_cv*scale_pitch;
						time_stamp_cv         = time_cv;
						
						//pitch_cv_filtered = LowPassFilter1p_Update(&lpf_pitch_cv, pitch_command_from_cv);
						
						if (time_stamp_cv != time_stamp_cv_prev){
							gimbal.r_x[0] = gimbal.x[0] + yaw_command_from_cv;
							gimbal.r_x[1] = cm_p_des_origin + pitch_command_from_cv;
//						// dt between last two CV frames (seconds)
//						float cv_dt = time_stamp_cv - time_stamp_cv_prev;

//						if (cv_dt > 0.0f) {
//								// Use a counter or a running timestamp here
//								float t_now = HAL_GetTick();
//								float alpha = (t_now - time_stamp_cv_prev) / cv_dt;
//								alpha = fminf(fmaxf(alpha, 0.0f), 1.0f);

//								// Interpolate the relative command
//								float yaw_interp   = yaw_command_from_cv_prev + alpha * (yaw_command_from_cv - yaw_command_from_cv_prev);
//								float pitch_interp = pitch_command_from_cv_prev + alpha * (pitch_cv_filtered - pitch_command_from_cv_prev);

//								// Convert relative ? absolute reference
//								gimbal.r_x[0] = gimbal.x[0] + yaw_interp;
//								gimbal.r_x[1] = cm_p_des_origin + pitch_interp;
//						} else {
//								// No valid CV interval yet — hold current position
//								gimbal.r_x[0] = gimbal.x[0];
//								gimbal.r_x[1] = cm_p_des_origin;
//						}
//						break;
				
			}
        default:
            break;
    }

	saturate_in_range(&gimbal.r_x[1],-0.5f,0.0f);
  /*****************************/
 /*   CONTROL LOOP EXECUTION  */
/*****************************/

	gimbal.u[0] = PID_Calculate(&pid_yaw_pos, gimbal.r_x[0], gimbal.x[0]);
	float data[] = { gimbal.r_x[0], gimbal.x[0], gimbal.u[0] };
	RTT_Log(data, 3);

}