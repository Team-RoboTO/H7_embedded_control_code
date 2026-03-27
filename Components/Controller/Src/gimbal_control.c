/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : gimbal_control.c
  * @brief          : Control of yaw and pitch motors 
  ******************************************************************************
  */
/* USER CODE END Header */

#include "gimbal_control.h"
#include "stdbool.h"
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include "control_utils.h"
#include "INS_task.h"
#include "math_utils.h"
#include "Remote_Control.h"
#include "robot_config.h"
#include "Motor.h"
#include "chassis_control.h"
#include "state_machine.h"
#include "PID.h"
#include "cubemars_motor.h"
#include "damiao_motor.h"
#include "MiniPC.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t gimbal = {
    .n          = 4,
    .m          = 2,
    .p          = 4,
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

/**
 * @brief PID controllers using PID_Info_TypeDef
 * 
 * Parameter array layout (PID_PARAMETER_NUM = 7):
 * [0] KP, [1] KI, [2] KD, [3] Alpha (LPF), [4] Deadband, [5] LimitIntegral, [6] LimitOutput
 */

PID_Info_TypeDef pid_yaw_pos;
PID_Info_TypeDef pid_yaw_vel;

// Yaw Position PID params: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
float pid_yaw_pos_params[PID_PARAMETER_NUM] = {4.0f, 0.01f, 1.0f, 0.0f, 0.0f, 10.0f, 30.0f};

// Yaw Velocity PID params: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
float pid_yaw_vel_params[PID_PARAMETER_NUM] = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 30.0f};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

static float remote_commands_yaw;
static float remote_commands_pitch;

static float time_stamp_cv = 0;
static float yaw_command_from_cv = 0;
static float pitch_command_from_cv = 0;
static float time_stamp_cv_prev = 0;
static float yaw_command_from_cv_prev;
static float pitch_command_from_cv_prev;

static uint8_t is_first_iter = true;

static float m_linear_interpolation_yaw = 0;
static float m_linear_interpolation_pitch = 0;
static float yaw_sat = 0;
static float pitch_sat = 0;

static float MIT_p_des = 0.0f;
static float MIT_v_des = 0.0f;
static float MIT_kp = 45.0f;
static float MIT_kd = 1.0f;
static float MIT_t_ff = 0.0f;
uint16_t ID_pitch = 106;

float cm_p_des_origin = 0;

extern uint8_t is_rotating;

float test_angle = 0;

float OVER_ESTIMATED_CV_FREQUENCY = 90;


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
    gimbal.x[2] = INS_Info.Yaw_Gyro;                   // yaw velocity   [rad/s]

    // update reference history
    for (uint8_t i = 0; i < 2; i++) {
        gimbal.r_x_prev[i] = gimbal.r_x[i];
    }
		
    // one-time initialization
    if (is_first_iter) {
        gimbal.r_x[0] = gimbal.x[0];
        cm_p_des_origin =  INS_Info.Roll_Angle * DEG_TO_RAD;
			
				// PID_INIT
				PID_Init(&pid_yaw_pos, PID_POSITION, pid_yaw_pos_params);
				PID_Init(&pid_yaw_vel, PID_VELOCITY, pid_yaw_vel_params);
			
        is_first_iter = false;
    }
		
		// STOP command: zero all outputs and reset
    if (state_remote_commands == COMMANDS_STOP) {
				
				// reset both PIDs on stop
				pid_yaw_pos.PID_Calc_Clear(&pid_yaw_pos);
				pid_yaw_vel.PID_Calc_Clear(&pid_yaw_vel);
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
													gimbal.r_x[0] = gimbal.x[0] + (remote_commands_yaw / MAX_RC_TILT) * 45 * DEG_TO_RAD;
												}
                        test_angle = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 20 * DEG_TO_RAD;
                    break;

                case COMMANDS_KEYBOARD_MOUSE:						
                    remote_commands_yaw   = RC_info.Mouse.X*0.001;
                    remote_commands_pitch = RC_info.Mouse.Y*0.001;
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

						yaw_command_from_cv   = yaw_cv;
						pitch_command_from_cv = pitch_cv;
						time_stamp_cv         = time_cv;

						// dt between last two CV frames (seconds)
						float cv_dt = time_stamp_cv - time_stamp_cv_prev;

						if (cv_dt > 0.0f) {
								// Use a counter or a running timestamp here
								float t_now = HAL_GetTick();
								float alpha = (t_now - time_stamp_cv_prev) / cv_dt;
								alpha = fminf(fmaxf(alpha, 0.0f), 1.0f);

								// Interpolate the relative command
								float yaw_interp   = yaw_command_from_cv_prev + alpha * (yaw_command_from_cv - yaw_command_from_cv_prev);
								float pitch_interp = pitch_command_from_cv_prev + alpha * (pitch_command_from_cv - pitch_command_from_cv_prev);

								// Convert relative ? absolute reference
								gimbal.r_x[0] = gimbal.x[0] + yaw_interp;
								gimbal.r_x[1] = cm_p_des_origin + pitch_interp;
						} else {
								// No valid CV interval yet — hold current position
								gimbal.r_x[0] = gimbal.x[0];
								gimbal.r_x[1] = cm_p_des_origin;
						}
						break;
				

        default:
            break;
    }

  /*****************************/
 /*   CONTROL LOOP EXECUTION  */
/*****************************/

	gimbal.u[0] = PID_Calculate(&pid_yaw_pos, gimbal.r_x[0], gimbal.x[0]);

}