#include "shooting_control.h"
#include "PID.h"
#include "motor.h"
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

  /*******************/
 /*   CONTROLLERS   */
/*******************/

/**
 * Parameter array layout (PID_PARAMETER_NUM = 7):
 * [0] KP, [1] KI, [2] KD, [3] Alpha (LPF), [4] Deadband, [5] LimitIntegral, [6] LimitOutput
 */

static PID_Info_TypeDef pid_shoot_wheel_left;
static PID_Info_TypeDef pid_shoot_wheel_right;
static PID_Info_TypeDef pid_rev_pos;
static PID_Info_TypeDef pid_rev_vel;

// Shoot wheel velocity PIDs: KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput
static float pid_shoot_wheel_left_params[PID_PARAMETER_NUM]  = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f};
static float pid_shoot_wheel_right_params[PID_PARAMETER_NUM] = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f};

// REV position PID (outer loop): KP is overwritten at runtime per shooting mode
static float pid_rev_pos_params[PID_PARAMETER_NUM] = {27.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 200.0f};

// REV velocity PID (inner loop)
static float pid_rev_vel_params[PID_PARAMETER_NUM] = {7.0f,  0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10000.0f};

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

float r_shoot_wheels_ang_vel = 200;  // [rad/s]
uint16_t ciao = 0;
static uint8_t need_to_set_rev_ang_pos_reference = true;
static float rev_shooting_frequency = 20;  // Bullets per second [Hz]

static float gain_overall_shoot_wheels = 1.0f;
static float gain_overall_rev          = 1.0f;
static uint8_t is_first_iter          = true;
bool unstuck_rev_enabled              = 0;

// Runtime-adjustable Kp values for REV position loop per shooting mode
static float Kp_pid_rev_pos_single_shooting = 27.0f;
static float Kp_pid_rev_pos_triple_shooting = 9.0f;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_shooting(void)
{
    _control_loop_shoot_wheels();
    //_control_loop_rev();

    is_first_iter = false;

//    CAN_Tx_shoot_wheels_rev(
//        (int16_t) shoot_wheels_and_rev.u[0],
//        (int16_t) shoot_wheels_and_rev.u[1],
//        (int16_t) shoot_wheels_and_rev.u[2]
//    );
	
	  FDCAN2_TxFrame.Header.Identifier = 0x200;
		//Control_Info.SendValue[0] = 2000;
    FDCAN2_TxFrame.Data[0] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[0] >> 8);
		FDCAN2_TxFrame.Data[1] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[0]);
		FDCAN2_TxFrame.Data[2] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[1] >> 8);
		FDCAN2_TxFrame.Data[3] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[1]);
		FDCAN2_TxFrame.Data[4] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[2] >> 8); // motor 3
		FDCAN2_TxFrame.Data[5] = (uint8_t)((int16_t) shoot_wheels_and_rev.u[2]);
		FDCAN2_TxFrame.Data[6] = (uint8_t)(0x00); // motor 4
		FDCAN2_TxFrame.Data[7] = (uint8_t)(0x00);

   USER_FDCAN_AddMessageToTxFifoQ(&FDCAN2_TxFrame);
}

  /*********************************/
 /*   SHOOT WHEELS CONTROL LOOP   */
/*********************************/

void _control_loop_shoot_wheels(void)
{
		if (is_first_iter == 1) {
			PID_Init(&pid_shoot_wheel_left,  PID_POSITION, pid_shoot_wheel_left_params);
			PID_Init(&pid_shoot_wheel_right, PID_POSITION, pid_shoot_wheel_right_params);
		}
	
    // Update state from sensors
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[0] = (float) shooting_motor[0].Data.Velocity*2*pi/60;  // Left wheel angular velocity  [rad/s]
    shoot_wheels_and_rev.x[1] = (float) shooting_motor[1].Data.Velocity*2*pi/60;  // Right wheel angular velocity [rad/s]

    // Update reference history
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.r_x_prev[i] = shoot_wheels_and_rev.r_x[i];
    }

    // Set velocity setpoints based on state machine
    switch (ciao) {
        case 1:
            shoot_wheels_and_rev.r_x[0] = +r_shoot_wheels_ang_vel;
            shoot_wheels_and_rev.r_x[1] = -r_shoot_wheels_ang_vel;
            break;
        case SHOOT_WHEELS_STOP:
        default:
            shoot_wheels_and_rev.r_x[0] = 0;
            shoot_wheels_and_rev.r_x[1] = 0;
            break;
    }

//    // Hard override on STOP command
//    if (state_remote_commands == COMMANDS_STOP) {
//        shoot_wheels_and_rev.r_x[0] = 0;
//        shoot_wheels_and_rev.r_x[1] = 0;

//        // Reset PIDs to avoid windup after stop
//        pid_shoot_wheel_left.PID_Calc_Clear(&pid_shoot_wheel_left);
//        pid_shoot_wheel_right.PID_Calc_Clear(&pid_shoot_wheel_right);
//    }

    // PID_Calculate(pid, Target, Measure) handles error/integral/derivative internally
    shoot_wheels_and_rev.u[0] = PID_Calculate(&pid_shoot_wheel_left,
                                               shoot_wheels_and_rev.r_x[0],
                                               shoot_wheels_and_rev.x[0]);

    shoot_wheels_and_rev.u[1] = PID_Calculate(&pid_shoot_wheel_right,
                                               shoot_wheels_and_rev.r_x[1],
                                               shoot_wheels_and_rev.x[1]);

    // ADC conversion and output saturation
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.u[i] *= M3508_ADC_CONVERTION;
        saturate(&shoot_wheels_and_rev.u[i], 10000);
    }

#if !IS_SHOOT_WHEELS_ENABLED
    shoot_wheels_and_rev.u[0] = 0;
    shoot_wheels_and_rev.u[1] = 0;
#endif
}

  /************************/
 /*   REV CONTROL LOOP   */
/************************/

void _control_loop_rev(void)
{
    // On STOP: zero output and reset PIDs
    if (state_remote_commands == COMMANDS_STOP) {
        shoot_wheels_and_rev.u[2] = 0;
        pid_rev_pos.PID_Calc_Clear(&pid_rev_pos);
        pid_rev_vel.PID_Calc_Clear(&pid_rev_vel);
        return;
    }

    // Update state from sensors
    for (uint8_t i = 2; i < shoot_wheels_and_rev.p; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[2] = (float) 0; //M2006_rev.cumulative_ang_pos_rad;      // REV angular position [rad]
    shoot_wheels_and_rev.x[3] = (float) shooting_motor[2].Data.Velocity;       // REV angular velocity [rad/s]

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

    // Clear unstuck flag once error is small enough
    if (unstuck_rev_enabled && fabs(shoot_wheels_and_rev.r_x[2] - shoot_wheels_and_rev.x[2]) < 1 * pi / 180) {
        unstuck_rev_enabled = 0;
    }

    // Setpoint generation based on REV state machine
    switch (state_rev) {

        case REV_UNSTUCK:
            if (!unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] - 45 * pi / 180;
                unstuck_rev_enabled = 1;
            }
            break;

        case REV_STOP:
            shoot_wheels_and_rev.r_x[2] = 0;
            need_to_set_rev_ang_pos_reference = true;
            break;

        case REV_SINGLE_SHOOTING:
            if (need_to_set_rev_ang_pos_reference && !unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] + pi / 4;
                need_to_set_rev_ang_pos_reference = false;
            }
            break;

        case REV_TRIPLE_SHOOTING:
            if (need_to_set_rev_ang_pos_reference && !unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] + 3 * pi / 4;
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

    // Adjust REV position PID gain depending on shooting mode.
    // NOTE: PID_Info_TypeDef does not expose Kp directly at runtime,
    // so we re-initialize only the parameter struct field and re-run PID_Param_Init
    // to apply the updated gain without resetting the integral or error history.
    switch (state_rev) {
        case REV_TRIPLE_SHOOTING:
            pid_rev_pos.Param.KP = Kp_pid_rev_pos_triple_shooting;
            break;
        case REV_SINGLE_SHOOTING:
        default:
            pid_rev_pos.Param.KP = Kp_pid_rev_pos_single_shooting;
            break;
    }

    // --- OUTER LOOP: Position Control ---
    // Output is the desired REV velocity setpoint
    shoot_wheels_and_rev.r_x[3] = PID_Calculate(&pid_rev_pos,
                                                  shoot_wheels_and_rev.r_x[2],
                                                  shoot_wheels_and_rev.x[2]);

    // In MULTIPLE_SHOOTING the velocity reference is set directly by the state machine,
    // so we override the position PID output in that case
    if (state_rev == REV_MULTIPLE_SHOOTING) {
        shoot_wheels_and_rev.r_x[3] = rev_shooting_frequency * pi / 4;
    }

    // --- INNER LOOP: Velocity Control ---
    // Output is the motor current command
    shoot_wheels_and_rev.u[2] = PID_Calculate(&pid_rev_vel,
                                               shoot_wheels_and_rev.r_x[3],
                                               shoot_wheels_and_rev.x[3]);

    // Apply overall gain, ADC conversion and output saturation
    shoot_wheels_and_rev.u[2] *= gain_overall_rev;
    shoot_wheels_and_rev.u[2] *= M2006_ADC_CONVERTION;
    saturate(&shoot_wheels_and_rev.u[2], 10000);

#if !IS_REV_ENABLED
    shoot_wheels_and_rev.u[2] = 0;
#endif
}