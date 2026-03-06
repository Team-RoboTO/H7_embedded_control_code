#include "gimbal_control.h"

#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include "control_utils.h"
#include "CAN_receive.h"
#include "bmi088driver.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "CAN_receive.h"
#include "math_utils.h"
#include "remote_commands.h"
#include "AI_receive.h"
#include "robot_config.h"
#include "gimbal_task.h"
#include "FreeRTOS.h"
#include "motors.h"
#include "motors_std_circ.h"
#include "can_transmit_std_circ.h"
#include "control_std_circ_chassis.h"
#include "mouse_keyboard_commands.h"
#include "pid_controller.h"
#include "state_machine_std_circ.h"

/**
 * @brief Gimbal Control System for Standard Circular Robot
 * 
 * This file implements the control logic for a 2-axis gimbal (yaw and pitch).
 * It handles:
 * - sensor data acquisition (IMU/INS) to know where the gimbal is pointing
 * - reference/setpoint generation (manual and auto-aim)
 * - cascaded PID control loops (position -> velocity)
 * - actuation command generation (voltage for yaw, MIT Mode for pitch)
 * 
 * This control structure uses a state-space representation in which:
 * - x[0]: Yaw Position
 * - x[1]: Pitch Position
 * - x[2]: Yaw Velocity
 * - x[3]: Pitch Velocity
 */

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

 // System State Initialization
controlled_system_t std_circ_gimbal = {
    
    .n          = 4,  // number of system states (yaw and pitch angle, yaw and pitch speed)
    .m          = 2,  // number of system inputs (yaw and pitch motor commands)
    .p          = 4,  // number of system outputs (measured pos/vel for both axes, from each sensor)
    .x          = {0},
    .x_prev     = {0},
    .u          = {0},
    .u_prev     = {0},
    .r_x        = {0},
    .r_x_prev   = {0},
    .e_x        = {0},
    .e_x_prev   = {0},
    .ei_x       = {0},
    .ed_x       = {0}
};

  /*******************/
 /*   CONTROLLERS   */
/*******************/

/**
 * @brief Yaw Position PID Controller
 * Outer loop controller. Calculates desired velocity based on position error.
 */
static pid__t pid_yaw_pos = {
    
    .Kp = 5, // 20
	.Ki = 2,
	.Kd = 0,
    .u = 0,
    .up = 0,
    .ui = 0,
    .ud = 0,
    .lpf_up = NULL,
    .lpf_ui = NULL,
    .lpf_ud = NULL,
    .saturation_up = NULL,
    .saturation_ui = NULL,
    .saturation_ud = NULL
};

/**
 * @brief Yaw Velocity PID Controller
 * Inner loop controller. Calculates motor current (voltage) based on velocity error.
 */
static pid__t pid_yaw_vel = {
    
    .Kp = 10, //4,
	.Ki = 0,
	.Kd = 0,
    .u = 0,
	.up = 0,
	.ui = 0,
	.ud = 0,
    .lpf_up = NULL,
    .lpf_ui = NULL,
    .lpf_ud = NULL,
    .saturation_up = NULL,
    .saturation_ui = NULL,
    .saturation_ud = NULL
};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

static int16_t remote_commands_yaw;
static int16_t remote_commands_pitch;

static fp32 yaw_command_from_cv = 0;
static fp32 pitch_command_from_cv = 0;
static fp32 yaw_command_from_cv_prev;
static fp32 pitch_command_from_cv_prev;

static fp32 saturation_ei_pitch_pos = 0.5;  // [rad*s] - saturation for pitch position integral term

static uint8_t is_first_iter = TRUE;
static fp32 m_linear_interpolation_yaw = 0;
static fp32 m_linear_interpolation_pitch = 0;
static fp32 yaw_sat = 0;
static fp32 pitch_sat = 0;

/**
 * @brief  Cubemars MIT mode control variables
 * 
 * "MIT Mode" is the control protocol implemented in the Cubemars actuator. 
 * The controller sends 5 values (instead of voltage or current):
 * 1. P_des: Desired Position
 * 2. V_des: Desired Velocity 
 * 3. Kp: Proportional Gain (value of the motor resistance to position errors)
 * 4. Kd: Derivative Gain (value of the motor resistance to velocity)
 * 5. T_ff: Feedforward Torque (Extra torque to overcome gravity or dynamics)
 * 
 * The torque is calculated internally by the motor driver with:
 * T = Kp * (P_des - P_meas) + Kd * (V_des - V_meas) + T_ff
 */

static float MIT_p_des = 0.0f; // range -12.5 - +12.5 [rad]
static float MIT_v_des = 0.0f; // range -45.0 + 45.0 [rad/s]
static float MIT_kp = 45.0f;   // range 0-500
static float MIT_kd = 1.0f;    // range 0-5
static float MIT_t_ff = 0.0f;  // range -15.0 - 15.0 [Nm]
uint16_t ID_pitch = 106;

fp32 cm_p_des_origin= 0;
fp32 sat_ui = 0;

extern uint8_t is_rotating;

  /********************/
 /*   CONTROL LOOP   */
/********************/

/**
 * @brief Main Control Loop Function
 * 
 * This function is the entire control process, in steps:
 * 1. checks for STOP commands.
 * 2. updates system state (x) from sensor inputs (y).
 * 3. initializes references on startup.
 * 4. determines desired setpoints (r_x) based on manual or auto-aim mode.
 * 5. computes errors (e_x) and derivatives.
 * 6. runs PID controllers for position and velocity loops.
 * 7. formats and transmits commands to motors.
 */
void control_loop_gimbal() {
    
// if a stop command arrives, send zeros as control signals
    if (state_remote_commands == COMMANDS_STOP) {
        
                // alternating logic to safely exit control modes
                switch (iteration_number){
                        case 0: 
                                // exit MIT Mode (pitch motor)
                                CAN_Tx_MIT_Exit_Control_Mode(ID_pitch);
                                iteration_number=1;
                                is_first_iter = 1;  // reset init flag
                                break;
                        case 1:
                                // send zero voltage (yaw motor)
                                CAN_Tx_gimbal((int16_t) 0, 0);
                                iteration_number=0;
                                break;
                        default:
                                return;
                }             
        return;
    }
	
	// update outputs from sensor data
    // copy current output to previous output for history
    for (uint8_t i = 0; i < std_circ_gimbal.p; i++) {
        std_circ_gimbal.y_prev[i] = std_circ_gimbal.y[i];
    }
    // read new sensor values from INS (Inertial Navigation System) and IMU (Gyroscope)
    std_circ_gimbal.y[0] = (fp32) ins_correct_angle[2] * DEG_TO_RAD;  // yaw position of uC's IMU [rad]
    std_circ_gimbal.y[1] = (fp32) ins_correct_angle[1] * DEG_TO_RAD;  // pitch position of uC's IMU [rad]
    std_circ_gimbal.y[2] = (fp32) gz;  // yaw velocity of uC's IMU [rad/s]
    std_circ_gimbal.y[3] = (fp32) gy;  // pitch velocity of uC's IMU [rad/s]
    
    // update states
    // the state (x) is just the filtered version of the sensor output (y)
    for (uint8_t i = 0; i < std_circ_gimbal.n; i++) {
        std_circ_gimbal.x_prev[i] = std_circ_gimbal.x[i];
    }
    std_circ_gimbal.x[0] = std_circ_gimbal.y[0];
    std_circ_gimbal.x[1] = std_circ_gimbal.y[1];
    std_circ_gimbal.x[2] = std_circ_gimbal.y[2];
    std_circ_gimbal.x[3] = std_circ_gimbal.y[3];
    
    // update references
    for (uint8_t i = 0; i < 2; i++) {
        std_circ_gimbal.r_x_prev[i] = std_circ_gimbal.r_x[i];
    }
    
    // one-time initialization logic
    if (is_first_iter) {
        // set initial yaw reference to current yaw
        std_circ_gimbal.r_x[0] = std_circ_gimbal.x[0]; 
            
        // capture the starting pitch angle as the setpoint (origin) for relative movement
        cm_p_des_origin = -ins_correct_angle[0] * DEG_TO_RAD;
        
        // send commands to enter MIT Control Mode and set the position to zero
        CAN_Tx_MIT_Enter_Control_Mode(ID_pitch);
        CAN_Tx_MIT_Set_Zero_Position(ID_pitch);
    
        is_first_iter = FALSE;              
    }

    // state machine for control logic (manual vs auto-aim)
    switch (state_gimbal) {
        
        case GIMBAL_MANUAL_AIM:
            
            // manual aim driven by remote commands
            switch (state_remote_commands) {
                
                case COMMANDS_REMOTE_CONTROLLER:
                    // update commands from remote controller joysticks
                    remote_commands_yaw     = - remote_controller_right_joystick_horizontal;
                    remote_commands_pitch   = + remote_controller_right_joystick_vertical;

                    // increment/decrement yaw reference based on joystick input
                    if (remote_commands_yaw != 0) {
                        std_circ_gimbal.r_x[0] = std_circ_gimbal.x[0] + (remote_commands_yaw / MAX_RC_TILT) * 45 * DEG_TO_RAD;  // move yaw setpoint based on joystick
                    }
                    // update pitch setpoint for MIT Mode (relative to origin)
                    if (remote_commands_pitch != 0) {
                        cm_p_des = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 30 * DEG_TO_RAD;  
                    }
                    break;
                
                case COMMANDS_KEYBOARD_MOUSE:
                    // update commands from mouse input
                    remote_commands_yaw     = yaw_command_mouse_to_remote_controller(dt_gimbal);
                    remote_commands_pitch   = pitch_command_mouse_to_remote_controller(dt_gimbal);
                    std_circ_gimbal.r_x[0] += (remote_commands_yaw / MAX_RC_TILT) * 15 * DEG_TO_RAD;
                    // pitch is managed only by Cubemars MIT mode
                    cm_p_des = cm_p_des_origin + (remote_commands_pitch / MAX_RC_TILT) * 30 * DEG_TO_RAD;  
                    break;
                
                default:
                    break;
            }
            
            break;
        
        case GIMBAL_AUTO_AIM:
            
		
            // auto-aim driven by Computer Vision (CV) commands
			yaw_command_from_cv_prev   = yaw_command_from_cv;
			pitch_command_from_cv_prev = pitch_command_from_cv;
			yaw_command_from_cv   = yaw_cv;
			pitch_command_from_cv = pitch_cv;

        
            // update internal references for the pitch (cm_p_des is not updated though)
			if (pitch_command_from_cv != pitch_command_from_cv_prev)
			{
				// smooth transition logic
                // instead of jumping instantly to the new target, predict the movement path: results in smoother movement
				m_linear_interpolation_yaw = yaw_command_from_cv * OVER_ESTIMATED_CV_FREQUENCY;
				m_linear_interpolation_pitch = pitch_command_from_cv * OVER_ESTIMATED_CV_FREQUENCY;
				
				std_circ_gimbal.r_x[0] = std_circ_gimbal.x[0];
				std_circ_gimbal.r_x[1] = std_circ_gimbal.x[1];
				
				yaw_sat = std_circ_gimbal.x[0] + yaw_command_from_cv;
				pitch_sat = std_circ_gimbal.x[1] + pitch_command_from_cv;
			}
            
            // advance the setpoint along the predicted path
			std_circ_gimbal.r_x[0] += m_linear_interpolation_yaw * dt_gimbal;
			std_circ_gimbal.r_x[1] += m_linear_interpolation_pitch * dt_gimbal;
			
            // saturate pitch and yaw to not overshoot the target
			saturate(&std_circ_gimbal.r_x[0], yaw_sat);
			saturate(&std_circ_gimbal.r_x[1], pitch_sat);

            break;
        
        default:
            break;
    }
    
    saturate_in_range(&std_circ_gimbal.r_x[1], -23 * DEG_TO_RAD, +19.5 * DEG_TO_RAD);

		if(!is_rotating) pid_yaw_pos.saturation_ui = &sat_ui;
		else pid_yaw_pos.saturation_ui = NULL;

    
  /*****************************/
 /*   CONTROL LOOP EXECUTION  */
/*****************************/
		
    // update errors w.r.t. position states (yaw: index 0, pitch: index 1)
    for (uint8_t i = 0; i < 2; i++) {
        std_circ_gimbal.e_x_prev[i]  = std_circ_gimbal.e_x[i];
        std_circ_gimbal.e_x[i]       = std_circ_gimbal.r_x[i] - std_circ_gimbal.x[i];
        std_circ_gimbal.ei_x[i]     += std_circ_gimbal.e_x[i] * dt_gimbal;
        std_circ_gimbal.ed_x[i]      = (std_circ_gimbal.e_x[i] - std_circ_gimbal.e_x_prev[i]) / dt_gimbal;
    }
    
    // anti-windup: saturate integral term for pitch
    saturate(&std_circ_gimbal.ei_x[1], saturation_ei_pitch_pos);
	
//	if (std_circ_gimbal.e_x[0] > 0)
//	{
//	
//		pid_yaw_pos.Kp = 21;
//		pid_yaw_vel.Kp = 1.5;
//		
//		if (state_chassis == CHASSIS_CONTIGUOUS_ROTATION)
//		{
//			pid_yaw_pos.Kp = 35;
//			pid_yaw_vel.Kp = 1.5;
//		}
//		if(state_chassis == CHASSIS_CONTIGUOUS_ROTATION && state_gimbal == GIMBAL_AUTO_AIM )
//		{
//			pid_yaw_pos.Kp = 50;
//			pid_yaw_vel.Kp = 3.5;
//		}	

//	}
//	else
//	{
//		pid_yaw_pos.Kp = 17;
//		pid_yaw_vel.Kp = 1.25;
//	}
    
    // outer control loop: POSITION Control
    // calculates desired velocity (output 'u') based on position error
    pid_control(&pid_yaw_pos,   std_circ_gimbal.e_x[0], std_circ_gimbal.ei_x[0], std_circ_gimbal.ed_x[0]);  // yaw PID
    pid_control(&pid_pitch_pos, std_circ_gimbal.e_x[1], std_circ_gimbal.ei_x[1], std_circ_gimbal.ed_x[1]);  // pitch PID
    
    // the output of position PID becomes the reference (setpoint) for the velocity PID
    std_circ_gimbal.r_x[2] = pid_yaw_pos.u;
    std_circ_gimbal.r_x[3] = pid_pitch_pos.u;
    
    // update errors w.r.t. velocity states (yaw: index 2, pitch: index 3)
    for (uint8_t i = 2; i < 4; i++) {
        std_circ_gimbal.e_x_prev[i]  = std_circ_gimbal.e_x[i];
        std_circ_gimbal.e_x[i]       = std_circ_gimbal.r_x[i] - std_circ_gimbal.x[i];
        std_circ_gimbal.ei_x[i]     += std_circ_gimbal.e_x[i] * dt_gimbal;
        std_circ_gimbal.ed_x[i]      = (std_circ_gimbal.e_x[i] - std_circ_gimbal.e_x_prev[i]) / dt_gimbal;
    }
    
    // inner control loop: VELOCITY Control
    for (uint8_t i = 0; i < std_circ_gimbal.m; i++) {
        std_circ_gimbal.u_prev[i] = std_circ_gimbal.u[i];
    }
    pid_control(&pid_yaw_vel,   std_circ_gimbal.e_x[2], std_circ_gimbal.ei_x[2], std_circ_gimbal.ed_x[2]);
    pid_control(&pid_pitch_vel, std_circ_gimbal.e_x[3], std_circ_gimbal.ei_x[3], std_circ_gimbal.ed_x[3]);

    // store final control signals - voltage commands
    std_circ_gimbal.u[0] = pid_yaw_vel.u;
    std_circ_gimbal.u[1] = pid_pitch_vel.u;
    
    // gain overall + ADC + saturation of control signals
    std_circ_gimbal.u[0] *= gain_overall_u_yaw;
    std_circ_gimbal.u[1] *= gain_overall_u_pitch;
    
    std_circ_gimbal.u[0] *= GM6020_ADC_CONVERTION;
    std_circ_gimbal.u[1] *= M3508_ADC_CONVERTION;
    
    // limit output values to protect hardware
    saturate(&std_circ_gimbal.u[0], 15000);
    saturate(&std_circ_gimbal.u[1], 25000);
    lpf_apply_filter(&lpf_pitch,std_circ_gimbal.u[1]);
    
// send control signals
// the CAN bus bandwidth is limited, so we alternate messages:
// iteration 0: send pitch motor command (MIT Mode)
// iteration 1: send yaw motor command (Standard Voltage Mode)
#if IS_GIMBAL_ENABLED
    switch (iteration_number){
        case 0: 
            // pitch command - MIT Mode
            CAN_Tx_MIT_Control(
                cm_p_des, 
                cm_v_des, 
                cm_kp, 
                cm_kd, 
                cm_t_ff,
								ID_pitch);
            iteration_number=1;
            break;
        case 1:
            // yaw command (voltage mode), using PID output (std_circ_gimbal.u[0])
            CAN_Tx_gimbal(
                    (int16_t) std_circ_gimbal.u[0], 0);  // pitch sent as 0 here because handled in case 0                         
            iteration_number=0;
            break;
        default:
                return;
                } 
#endif
    
    is_first_iter = FALSE;
}