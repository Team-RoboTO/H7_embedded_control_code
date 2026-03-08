#include "shooting_control.h"
#include "PID.h"
#include "motor.h"
#include "state_machine.h"
#include "control_utils.h"
#include "string.h"
#include "referee.h"
#include "referee_alg.h"
#include "gimbal_task.h"
#include "mouse_keyboard_commands.h"
#include "shooting_task.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t shoot_wheels_and_rev = {
    
    .n          = 4,  // Number of system states
    .m          = 3,  // Number of system inputs
    .p          = 4,  // Number of system outputs
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

static pid__t pid_shoot_wheel_left = {
    
    .Kp = 0.1,
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

static pid__t pid_shoot_wheel_right = {
    
    .Kp = 0.1,
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

static pid__t pid_rev_pos = {
    
    .Kp = 27,
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

static pid__t pid_rev_vel = {
    
    .Kp = 7,
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

static fp32 Kp_pid_rev_pos_single_shooting = 27;
static fp32 Kp_pid_rev_pos_triple_shooting = 9;

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/

fp32 r_shoot_wheels_ang_vel = 680;  // 650 [rad/s]

static uint8_t need_to_set_rev_ang_pos_reference = TRUE;
static fp32 rev_shooting_frequency = 20;  // Bullets per second [Hz]

static fp32 gain_overall_shoot_wheels = 1.0;
static fp32 gain_overall_rev = 1.0;
static uint8_t is_first_iter = TRUE;
bool unstuck_rev_enabled  = 0;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_shooting(void) {
    
    // Shoot wheels and rev control loops
    _control_loop_shoot_wheels();
    _control_loop_rev();
    
    is_first_iter = FALSE;
    
    // Send control signals
    CAN_Tx_shoot_wheels_rev(
        (int16_t) shoot_wheels_and_rev.u[0],
        (int16_t) shoot_wheels_and_rev.u[1],
        (int16_t) shoot_wheels_and_rev.u[2]
    );
}

  /*********************************/
 /*   SHOOT WHEELS CONTROL LOOP   */
/*********************************/

void _control_loop_shoot_wheels(void) {
    
    // Update outputs from sensor data
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[0] = (fp32) M3508_shoot_wheel[0].ang_vel_rads;  // Angular velocity of left shoot wheel [rad/s]
    shoot_wheels_and_rev.x[1] = (fp32) M3508_shoot_wheel[1].ang_vel_rads;  // Angular velocity of right shoot wheel [rad/s]
    
    // Update references
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.r_x_prev[i] = shoot_wheels_and_rev.r_x[i];
    }
    
    switch (state_shoot_wheels) {
        
        case SHOOT_WHEELS_SPIN:
            // Stop shoot wheels
            shoot_wheels_and_rev.r_x[0] = + r_shoot_wheels_ang_vel;
            shoot_wheels_and_rev.r_x[1] = - r_shoot_wheels_ang_vel;
            break;
        
        case SHOOT_WHEELS_STOP:
            // Spin shoot wheels
            shoot_wheels_and_rev.r_x[0] = 0;
            shoot_wheels_and_rev.r_x[1] = 0;
            break;
        
        default:
            shoot_wheels_and_rev.r_x[0] = 0;
            shoot_wheels_and_rev.r_x[1] = 0;
            break;
    }
    
    if (state_remote_commands == COMMANDS_STOP) {
        // Hard set reference speed to 0 (to stop shoot wheels immediately when robot shuts down)
        shoot_wheels_and_rev.r_x[0] = 0;
        shoot_wheels_and_rev.r_x[1] = 0;
    }
    
    // Update errors w.r.t. states
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.e_x_prev[i]  = shoot_wheels_and_rev.e_x[i];
        shoot_wheels_and_rev.e_x[i]       = shoot_wheels_and_rev.r_x[i] - shoot_wheels_and_rev.x[i];
        shoot_wheels_and_rev.ei_x[i]     += shoot_wheels_and_rev.e_x[i] * dt_shooting;
        shoot_wheels_and_rev.ed_x[i]      = (shoot_wheels_and_rev.e_x[i] - shoot_wheels_and_rev.e_x_prev[i]) / dt_shooting;
    }
    
    // PID control
    pid_control(&pid_shoot_wheel_left,  shoot_wheels_and_rev.e_x[0], shoot_wheels_and_rev.ei_x[0], shoot_wheels_and_rev.ed_x[0]);
    pid_control(&pid_shoot_wheel_right, shoot_wheels_and_rev.e_x[1], shoot_wheels_and_rev.ei_x[1], shoot_wheels_and_rev.ed_x[1]);
    shoot_wheels_and_rev.u[0] = pid_shoot_wheel_left.u;
    shoot_wheels_and_rev.u[1] = pid_shoot_wheel_right.u;
    
    // Gain overall + ADC + Saturation of control signals
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels_and_rev.u[i] *= M3508_ADC_CONVERTION;
        saturate(&shoot_wheels_and_rev.u[i], 10000);
    }
    
#if !IS_SHOOT_WHEELS_ENABLED
    // Set shoot wheels control signal to 0
    memset(shoot_wheels_and_rev.u, 0, sizeof(shoot_wheels_and_rev.u));
#endif
}

  /************************/
 /*   REV CONTROL LOOP   */
/************************/

void _control_loop_rev(void) {
    
    // If stop command arrived, send zeros as control signals
    if (state_remote_commands == COMMANDS_STOP) {
        
        memset(shoot_wheels_and_rev.u[2], 0, sizeof(shoot_wheels_and_rev.u[2]));
        return;
    }
    
    // Update outputs from sensor data
    for (uint8_t i = 2; i < shoot_wheels_and_rev.p; i++) {
        shoot_wheels_and_rev.x_prev[i] = shoot_wheels_and_rev.x[i];
    }
    shoot_wheels_and_rev.x[2] = (fp32) M2006_rev.cumulative_ang_pos_rad;  // Angular position of REV [rad/s]
    shoot_wheels_and_rev.x[3] = (fp32) M2006_rev.ang_vel_rads;  // Angular velocity of REV [rad/s]
    
    // Update references
    for (uint8_t i = 2; i < shoot_wheels_and_rev.n; i++) {
        shoot_wheels_and_rev.r_x_prev[i] = shoot_wheels_and_rev.r_x[i];
    }
    
    if (is_first_iter) {
        shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2];
    }
    
    if (unstuck_rev_enabled && fabs(shoot_wheels_and_rev.e_x[2]) < 1*pi/180)
    {
      unstuck_rev_enabled = 0;
    }
    
    
    switch (state_rev) {
        case REV_UNSTUCK:
            // Unstuck rev
            if (!unstuck_rev_enabled)
            {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] - 45*pi/180;
                unstuck_rev_enabled = 1;
            }
            break;
        
        case REV_STOP:
            // Stop REV and keep previous position reference
            shoot_wheels_and_rev.r_x[2] = 0;
            need_to_set_rev_ang_pos_reference = TRUE;
            break;
        
        case REV_SINGLE_SHOOTING:
            // Single shooting
            if (need_to_set_rev_ang_pos_reference && !unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] + pi/4;
                need_to_set_rev_ang_pos_reference = FALSE;
            }
            break;
        
        case REV_TRIPLE_SHOOTING:
            // Triple shooting
            if (need_to_set_rev_ang_pos_reference && !unstuck_rev_enabled) {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2] + 3*pi/4;
                need_to_set_rev_ang_pos_reference = FALSE;
            }
            break;
        
        case REV_MULTIPLE_SHOOTING:
            // Multiple shooting
            if(!unstuck_rev_enabled)
            {
                shoot_wheels_and_rev.r_x[2] = shoot_wheels_and_rev.x[2];
                shoot_wheels_and_rev.r_x[3] = rev_shooting_frequency * pi/4;
                need_to_set_rev_ang_pos_reference = FALSE;  
            }
            break;
        
        default:
            break;
    }
    
    // Set controllers gains
    switch (state_rev) {

        case REV_SINGLE_SHOOTING:
            // Single shooting
            pid_rev_pos.Kp = Kp_pid_rev_pos_single_shooting;
            break;
        
        case REV_TRIPLE_SHOOTING:
            // Triple shooting
            pid_rev_pos.Kp = Kp_pid_rev_pos_triple_shooting;
            break;
        
        default:
            // In any other case, use the single shooting gain
            pid_rev_pos.Kp = Kp_pid_rev_pos_single_shooting;
            break;
    }
    

    // Update errors w.r.t. position states
    shoot_wheels_and_rev.e_x_prev[2]  = shoot_wheels_and_rev.e_x[2];
    shoot_wheels_and_rev.e_x[2]       = shoot_wheels_and_rev.r_x[2] - shoot_wheels_and_rev.x[2];
    shoot_wheels_and_rev.ei_x[2]     += shoot_wheels_and_rev.e_x[2] * dt_shooting;
    shoot_wheels_and_rev.ed_x[2]      = (shoot_wheels_and_rev.e_x[2] - shoot_wheels_and_rev.e_x_prev[2]) / dt_shooting;
    
    // Outer control loop: position
    pid_control(&pid_rev_pos, shoot_wheels_and_rev.e_x[2], shoot_wheels_and_rev.ei_x[2], shoot_wheels_and_rev.ed_x[2]);
    
    if (state_rev != REV_MULTIPLE_SHOOTING) {
        shoot_wheels_and_rev.r_x[2] = pid_rev_pos.u;
    }
    
    // Update errors w.r.t. velocity states
    shoot_wheels_and_rev.e_x_prev[3]  = shoot_wheels_and_rev.e_x[3];
    shoot_wheels_and_rev.e_x[3]       = shoot_wheels_and_rev.r_x[3] - shoot_wheels_and_rev.x[3];
    shoot_wheels_and_rev.ei_x[3]     += shoot_wheels_and_rev.e_x[3] * dt_shooting;
    shoot_wheels_and_rev.ed_x[3]      = (shoot_wheels_and_rev.e_x[3] - shoot_wheels_and_rev.e_x_prev[3]) / dt_shooting;
    
    // Inner control loop: velocity
    pid_control(&pid_rev_vel, shoot_wheels_and_rev.e_x[3], shoot_wheels_and_rev.ei_x[3], shoot_wheels_and_rev.ed_x[3]);
    shoot_wheels_and_rev.u[2] = pid_rev_vel.u;
    
    shoot_wheels_and_rev.u[2] *= gain_overall_rev;
    shoot_wheels_and_rev.u[2] *= M2006_ADC_CONVERTION;
    saturate(&shoot_wheels_and_rev.u[2], 10000);
    
#if !IS_REV_ENABLED
    // Set rev control signal to 0
   memset(shoot_wheels_and_rev.u[2], 0, sizeof(shoot_wheels_and_rev.u));
#endif
}