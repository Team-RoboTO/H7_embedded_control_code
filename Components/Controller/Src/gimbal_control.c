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

static PID_Info_TypeDef pid_yaw_pos;
static PID_Info_TypeDef pid_yaw_vel;

// Yaw Position PID params: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
static float pid_yaw_pos_params[PID_PARAMETER_NUM] = {5.0f, 2.0f, 0.0f, 0.0f, 0.0f, 100.0f, 45.0f};

// Yaw Velocity PID params: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
static float pid_yaw_vel_params[PID_PARAMETER_NUM] = {10.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 15000.0f};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

static int16_t remote_commands_yaw;
static int16_t remote_commands_pitch;

static float yaw_command_from_cv = 0;
static float pitch_command_from_cv = 0;
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

  /****************************/
 /*   CONTROLLER INIT        */
/****************************/

/**
 * @brief Initialize all gimbal PID controllers.
 *        Call this once before starting the control loop.
 */
void gimbal_controllers_init(void)
{
    // Outer loop: position -> desired velocity
    PID_Init(&pid_yaw_pos, PID_POSITION, pid_yaw_pos_params);

    // Inner loop: velocity -> motor voltage
    PID_Init(&pid_yaw_vel, PID_POSITION, pid_yaw_vel_params);
    //
    // NOTE: Both use PID_POSITION mode because each loop outputs
    // an absolute command (not an increment):
    //   - pid_yaw_pos  outputs a desired velocity setpoint
    //   - pid_yaw_vel  outputs a voltage/current command
    // PID_VELOCITY would accumulate increments into the output,
    // which is not the intent for a cascaded position-velocity loop.
}

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_gimbal() {

    // STOP command: zero all outputs and reset
    if (state_remote_commands == COMMANDS_STOP) {
//				CAN_Tx_MIT_Exit_Control_Mode(ID_pitch);
//				// reset both PIDs on stop
//				pid_yaw_pos.PID_Calc_Clear(&pid_yaw_pos);
//				pid_yaw_vel.PID_Calc_Clear(&pid_yaw_vel);
//				CAN_Tx_gimbal((int16_t)0, 0);
//				is_first_iter = 1;
//        return;
    }

    // update system state from IMU/INS sensors
    for (uint8_t i = 0; i < gimbal.p; i++) {
        gimbal.x_prev[i] = gimbal.x[i];
    }
    gimbal.x[0] = INS_Info.Yaw_Angle * DEG_TO_RAD;     // yaw position  [rad]
    gimbal.x[1] = INS_Info.Pitch_Angle * DEG_TO_RAD;   // pitch position [rad]
    gimbal.x[2] = INS_Info.Yaw_Gyro;                   // yaw velocity   [rad/s]
    gimbal.x[3] = INS_Info.Pitch_Gyro;                   // pitch velocity [rad/s]

    // update reference history
    for (uint8_t i = 0; i < 2; i++) {
        gimbal.r_x_prev[i] = gimbal.r_x[i];
    }

    // one-time initialization
    if (is_first_iter) {
        gimbal.r_x[0] = gimbal.x[0];
        cm_p_des_origin =  INS_Info.Pitch_Angle * DEG_TO_RAD;
        is_first_iter = false;
    }

    // setpoint generation: manual or auto-aim
    switch (state_gimbal) {

        case GIMBAL_MANUAL_AIM:
            switch (state_remote_commands) {

                case COMMANDS_REMOTE_CONTROLLER:
                    remote_commands_yaw   = -RC_RIGHT_H;
                    remote_commands_pitch = +RC_RIGHT_V;
                    if (remote_commands_yaw != 0)
                        gimbal.r_x[0] = gimbal.x[0] + (remote_commands_yaw / MAX_RC_TILT) * 45 * DEG_TO_RAD;
                    if (remote_commands_pitch != 0)
                        //cm_p_des = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 30 * DEG_TO_RAD;
                    break;

                case COMMANDS_KEYBOARD_MOUSE:
                    remote_commands_yaw   = MOUSE_X_MOVE_SPEED*0.001;
                    remote_commands_pitch = MOUSE_Y_MOVE_SPEED*0.001;
                    gimbal.r_x[0] += (remote_commands_yaw   / MAX_RC_TILT) * 15 * DEG_TO_RAD;
                    //cm_p_des       = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 30 * DEG_TO_RAD;
                    break;

                default:
                    break;
            }
            break;

//        case GIMBAL_AUTO_AIM:
//            yaw_command_from_cv_prev   = yaw_command_from_cv;
//            pitch_command_from_cv_prev = pitch_command_from_cv;
//            yaw_command_from_cv        = yaw_cv;
//            pitch_command_from_cv      = pitch_cv;

//            if (pitch_command_from_cv != pitch_command_from_cv_prev) {
//                m_linear_interpolation_yaw   = yaw_command_from_cv   * OVER_ESTIMATED_CV_FREQUENCY;
//                m_linear_interpolation_pitch = pitch_command_from_cv * OVER_ESTIMATED_CV_FREQUENCY;
//                gimbal.r_x[0] = gimbal.x[0];
//                gimbal.r_x[1] = gimbal.x[1];
//                yaw_sat   = gimbal.x[0] + yaw_command_from_cv;
//                pitch_sat = gimbal.x[1] + pitch_command_from_cv;
//            }

//            gimbal.r_x[0] += m_linear_interpolation_yaw   * dt_gimbal;
//            gimbal.r_x[1] += m_linear_interpolation_pitch * dt_gimbal;
//            saturate(&gimbal.r_x[0], yaw_sat);
//            saturate(&gimbal.r_x[1], pitch_sat);
//            break;

        default:
            break;
    }

    saturate_in_range(&gimbal.r_x[1], -23 * DEG_TO_RAD, +19.5 * DEG_TO_RAD);

  /*****************************/
 /*   CONTROL LOOP EXECUTION  */
/*****************************/

    // --- OUTER LOOP: Position Control (yaw only) ---
    // PID_Calculate internally computes: error = Target - Measure
    // so we pass the reference and the measured position directly.
    // The output is a desired yaw velocity.
    gimbal.r_x[2] = PID_Calculate(&pid_yaw_pos, gimbal.r_x[0], gimbal.x[0]);

    // --- INNER LOOP: Velocity Control (yaw only) ---
    // The desired velocity from the position loop becomes the target here.
    // The output is a motor voltage command.
    for (uint8_t i = 0; i < gimbal.m; i++) {
        gimbal.u_prev[i] = gimbal.u[i];
    }
    gimbal.u[0] = PID_Calculate(&pid_yaw_vel, gimbal.r_x[2], gimbal.x[2]);

    // convert to ADC units and saturate
    gimbal.ud[0] = gimbal.u[0] * GM6020_ADC_CONVERTION;
    saturate(&gimbal.ud[0], 15000);

// transmit commands over CAN (alternating to respect bandwidth limits)
#if IS_GIMBAL_ENABLED
		// pitch - MIT Mode
//		CAN_Tx_MIT_Control(cm_p_des, cm_v_des, cm_kp, cm_kd, cm_t_ff, ID_pitch);

//		// yaw - voltage mode
//		CAN_Tx_gimbal((int16_t)gimbal.ud[0], 0);
#endif

    is_first_iter = false;
}