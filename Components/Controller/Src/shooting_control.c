#include "shooting_control.h"
#include "PID.h"
#include "DJI_motor.h"
#include "state_machine.h"
#include "control_utils.h"
#include "string.h"
#include "Referee_System.h"
#include "Remote_Control.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t shoot_wheels_and_rev = {
    .n          = 4,
    .m          = 3,
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

controlled_system_t lidar_lifter = {
    .n          = 1,
    .m          = 2,
    .p          = 1,
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

static PID_Info_TypeDef pid_shoot_wheel_left;
static PID_Info_TypeDef pid_shoot_wheel_right;
static PID_Info_TypeDef pid_rev_pos;
static PID_Info_TypeDef pid_rev_vel;
static PID_Info_TypeDef pid_ll_pos;
static PID_Info_TypeDef pid_ll_vel;

// Shoot wheel velocity PIDs: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
static float pid_shoot_wheel_left_params[PID_PARAMETER_NUM]  = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};
static float pid_shoot_wheel_right_params[PID_PARAMETER_NUM] = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};

// REV position PID (outer loop): KP is overwritten at runtime per shooting mode
static float pid_rev_pos_params[PID_PARAMETER_NUM] = {27.0f, 5.0f, 0.0f, 0.0f, 0.0f, 1.0f, 10000.0f};

// REV velocity PID (inner loop)
static float pid_rev_vel_params[PID_PARAMETER_NUM] = {7.0f,  1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 8.0f};

// LL position PID (outer loop): KP is overwritten at runtime per shooting mode
static float pid_ll_pos_params[PID_PARAMETER_NUM] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 10000.0f}; // TODO: tune

// LL velocity PID (inner loop)
static float pid_ll_vel_params[PID_PARAMETER_NUM] = {0.0f,  0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 8.0f}; // TODO: tune

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

float r_shoot_wheels_ang_vel                       = 600;  // [rad/s]
static uint8_t need_to_set_rev_ang_pos_reference   = true;
static float rev_shooting_frequency                = 10;  // Bullets per second [Hz]

static uint8_t is_first_iter                       = true;
bool unstuck_rev_enabled                           = 0;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_shooting(void)
{
    _control_loop_shoot_wheels();
    _control_loop_rev();
		_control_loop_lidar_lifter();

    is_first_iter = false;
		DJI_M3508_M2006_TxMessage(&FDCAN1_TxFrame, shoot_wheels_and_rev.ud[0], shoot_wheels_and_rev.ud[1], shoot_wheels_and_rev.ud[2], lidar_lifter.ud[0]);
}

  /*********************************/
 /*   SHOOT WHEELS CONTROL LOOP   */
/*********************************/

void _control_loop_shoot_wheels(void){
		
		if (is_first_iter == 1) {
			PID_Init(&pid_shoot_wheel_left,  PID_POSITION, pid_shoot_wheel_left_params);
			PID_Init(&pid_shoot_wheel_right, PID_POSITION, pid_shoot_wheel_right_params);
		}
		
		// STOP command
    if (state_remote_commands == COMMANDS_STOP) {
        shoot_wheels_and_rev.ud[0] = 0;
        shoot_wheels_and_rev.ud[1] = 0;

        // Reset PIDs to clean integral and avoid windup after stop
        pid_shoot_wheel_left.PID_Calc_Clear(&pid_shoot_wheel_left);
        pid_shoot_wheel_right.PID_Calc_Clear(&pid_shoot_wheel_right);
				
				return;
    }
		
	
    // Update state from sensors
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[0] = (float) DJI_Shooting_Motor[0].Data.Velocity_rads;  // Left wheel angular velocity  [rad/s]
    shoot_wheels_and_rev.x[1] = (float) DJI_Shooting_Motor[1].Data.Velocity_rads;  // Right wheel angular velocity [rad/s]

    // Update reference history
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.r_x_prev[i] = shoot_wheels_and_rev.r_x[i];
    }

    // Set velocity setpoints based on state machine
    switch (state_shoot_wheels) {
        case SHOOT_WHEELS_SPIN:
            shoot_wheels_and_rev.r_x[0] = +r_shoot_wheels_ang_vel;
            shoot_wheels_and_rev.r_x[1] = -r_shoot_wheels_ang_vel;
            break;
				
        case SHOOT_WHEELS_STOP:
					
        default:
            shoot_wheels_and_rev.r_x[0] = 0;
            shoot_wheels_and_rev.r_x[1] = 0;
            break;
    }

    // PID_Calculate(pid, Target, Measure) handles error/integral/derivative internally
    shoot_wheels_and_rev.u[0] = PID_Calculate(&pid_shoot_wheel_left, shoot_wheels_and_rev.r_x[0], shoot_wheels_and_rev.x[0]);

    shoot_wheels_and_rev.u[1] = PID_Calculate(&pid_shoot_wheel_right, shoot_wheels_and_rev.r_x[1], shoot_wheels_and_rev.x[1]);

    // ADC conversion and output saturation
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.ud[i] = shoot_wheels_and_rev.u[i]*DJI_Motor_ADC[DJI_M3508];
    }

}

  /************************/
 /*   REV CONTROL LOOP   */
/************************/

void _control_loop_rev(void)
{
    // Update state from sensors
    for (uint8_t i = 2; i < shoot_wheels_and_rev.p; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[2] = (float) DJI_Rev_Motor.Data.Angle_sum;           // REV angular position [rad]
    shoot_wheels_and_rev.x[3] = (float) DJI_Rev_Motor.Data.Velocity_rads;       // REV angular velocity [rad/s]

    // Update reference history
    for (uint8_t i = 2; i < shoot_wheels_and_rev.n; i++) {
        shoot_wheels_and_rev.r_x_prev[i] = shoot_wheels_and_rev.r_x[i];
    }

    // One-time init: latch current position as initial setpoint
    if (is_first_iter == 1) {
				PID_Init(&pid_rev_pos, PID_POSITION, pid_rev_pos_params);
				PID_Init(&pid_rev_vel, PID_POSITION, pid_rev_vel_params);
        shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2];
    }
		
		   // On STOP: zero output and reset PIDs
    if (state_remote_commands == COMMANDS_STOP) {
        shoot_wheels_and_rev.ud[2] = 0;
        pid_rev_pos.PID_Calc_Clear(&pid_rev_pos);
        pid_rev_vel.PID_Calc_Clear(&pid_rev_vel);
        return;
    }

    // Clear unstuck flag once error is small enough
    if (unstuck_rev_enabled && fabs(shoot_wheels_and_rev.r_x[2] - shoot_wheels_and_rev.x[2]) < 1 * pi / 180) {
        unstuck_rev_enabled = 0;
    }

    // Setpoint generation based on REV state machine
    switch (state_rev) {

        case REV_UNSTUCK:
            if (!unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] - 23 * pi / 180;
                unstuck_rev_enabled = 1;
            }
            break;

        case REV_STOP:
            shoot_wheels_and_rev.r_x[3] = 0;
            need_to_set_rev_ang_pos_reference = true;
            break;

        case REV_SINGLE_SHOOTING:
            if (need_to_set_rev_ang_pos_reference && !unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] + pi / 4;
                need_to_set_rev_ang_pos_reference = false;
            }
            break;

        case REV_MULTIPLE_SHOOTING:
            if (!unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2];
                shoot_wheels_and_rev.r_x[3] = rev_shooting_frequency * pi / 4;
                need_to_set_rev_ang_pos_reference = false;
            }
            break;

        default:
            break;
    }

    // --- OUTER LOOP: Position Control ---
    // Output is the desired REV velocity setpoint
    shoot_wheels_and_rev.r_x[3] = PID_Calculate(&pid_rev_pos, shoot_wheels_and_rev.r_x[2], shoot_wheels_and_rev.x[2]);

    // In MULTIPLE_SHOOTING the velocity reference is set directly by the state machine,
    // so we override the position PID output in that case
    if (state_rev == REV_MULTIPLE_SHOOTING) {
        shoot_wheels_and_rev.r_x[3] = rev_shooting_frequency * pi / 4;
    }

    // --- INNER LOOP: Velocity Control ---
    // Output is the motor current command
    shoot_wheels_and_rev.u[2] = PID_Calculate(&pid_rev_vel, shoot_wheels_and_rev.r_x[3], shoot_wheels_and_rev.x[3]);
		
	  shoot_wheels_and_rev.ud[2] = shoot_wheels_and_rev.u[2]*DJI_Motor_ADC[DJI_M2006]; //M2006_ADC_CONVERTION;
	
}


static float lidar_home_position = -1;
#define LIDAR_CURRENT_TRESHOLD 5000
static float CALIBRATION_SPEED =  0.2; // rad/s

void _control_loop_lidar_lifter(void)
{
			if (is_first_iter == 1) {
			PID_Init(&pid_ll_pos,  PID_POSITION, pid_ll_pos_params);
			PID_Init(&pid_ll_vel, PID_POSITION, pid_ll_vel_params);
			}
			
			lidar_lifter.x[0] = (float) DJI_Lidar_Motor.Data.Angle_sum;           // REV angular position [rad]
			lidar_lifter.x[1] = (float) DJI_Lidar_Motor.Data.Velocity_rads;
			
			float DJI_Lidar_current = (float) DJI_Lidar_Motor.Data.Current;

				
			if( lidar_home_position < 0 ){
				if( DJI_Lidar_Motor.Data.Current > LIDAR_CURRENT_TRESHOLD ){
					lidar_home_position = DJI_Lidar_Motor.Data.Angle_sum;
					lidar_lifter.r_x[0] = lidar_home_position; // Set home as base position
					lidar_lifter.r_x[1] = 0;
										
				} else {
					// Calcola pid con un incremento di 0.2 radianti al secondo
					lidar_lifter.r_x[1] = CALIBRATION_SPEED;
				}
				return;
			}
			
			switch (state_lidar_lifter) {
				case LIDAR_UP:
					lidar_lifter.r_x[0] = lidar_home_position; // to be modified
					break;
					
				case LIDAR_DOWN:
					lidar_lifter.r_x[0] = lidar_home_position;
					break;
			}
			
			// --- OUTER LOOP: Position Control ---
			// Output is the desired REV velocity setpoint
			lidar_lifter.u[0] = PID_Calculate(&pid_rev_pos, lidar_lifter.r_x[0], lidar_lifter.x[0]);

			// --- INNER LOOP: Velocity Control ---
			// Output is the motor current command
			lidar_lifter.u[1] = PID_Calculate(&pid_rev_vel, lidar_lifter.u[0], lidar_lifter.x[1]);
			lidar_lifter.ud[0] = lidar_lifter.u[0]*DJI_Motor_ADC[DJI_M2006]; //M2006_ADC_CONVERTION;
		
		return;
		
	
		
}