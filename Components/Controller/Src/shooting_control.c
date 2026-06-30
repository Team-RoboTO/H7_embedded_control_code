#include "shooting_control.h"
#include "PID.h"
#include "DJI_motor.h"
#include "Damiao_Motor.h"
#include "state_machine.h"
#include "control_utils.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t shoot_wheels = {
    .n = 2,  														 
    .m = 2,   													
    .p = 2,   													
};

controlled_system_t rev_and_push = {
    .n = 3,   													
    .m = 2,   													
    .p = 3,  														 
};

  /*******************/
 /*   CONTROLLERS   */
/*******************/
static PID_Info_TypeDef pid_shoot_wheel_left;
static PID_Info_TypeDef pid_shoot_wheel_right;

/* KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput */
static float pid_shoot_wheel_params[PID_PARAMETER_NUM] = {0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 10.0f};

#if IS_STD || IS_SENTRY
static PID_Info_TypeDef pid_rev_pos;
static PID_Info_TypeDef pid_rev_vel;

static float pid_rev_pos_params[PID_PARAMETER_NUM] = {27.0f, 5.0f, 0.0f, 0.0f, 0.0f, 1.0f, 10000.0f};
static float pid_rev_vel_params[PID_PARAMETER_NUM] = {7.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 8.0f};

#elif IS_HERO
static PID_Info_TypeDef pid_push_pos;
static PID_Info_TypeDef pid_push_vel;

/* KP, KI, KD, Alpha, Deadband, LimitIntegral, LimitOutput */
static float pid_push_pos_params[PID_PARAMETER_NUM] = {20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 20.0f};
static float pid_push_vel_params[PID_PARAMETER_NUM] = {7.0f, 1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 8.0f};
#endif

  /*************************/
 /*   CONTROL VARIABLES   */
/*************************/
static uint8_t need_to_set_rev_ang_pos_reference = true;
static bool is_first_iter = true;
static bool unstuck_rev_enabled = false;
float is_on_reset = 0;

#if IS_STD || IS_SENTRY
float r_shoot_wheels_ang_vel = 630.0f;   /* [rad/s] */
#elif IS_HERO
float r_shoot_wheels_ang_vel = 550.0f;   /* [rad/s] */
bool is_homing_rev = true;
static float vel_up_rev   = 10.0f;
static float vel_down_rev = 15.0f;

static uint8_t need_to_set_push_ang_pos_reference = true;
bool unstuck_push_enabled = false;
#endif

  /*************************/
 /*         INIT          */
/*************************/

void _shooting_control_init(void)
{
    /* Shoot wheels */
    PID_Init(&pid_shoot_wheel_left,  PID_POSITION, pid_shoot_wheel_params);
    PID_Init(&pid_shoot_wheel_right, PID_POSITION, pid_shoot_wheel_params);

    /* REV � latch current position as initial setpoint */
#if IS_STD || IS_SENTRY
    PID_Init(&pid_rev_pos, PID_POSITION, pid_rev_pos_params);
    PID_Init(&pid_rev_vel, PID_POSITION, pid_rev_vel_params);

    /* Sensor read needed before we can latch � do one read here */
    rev_and_push.x[0] = (float)DJI_Rev_Motor.Data.Angle_sum;
    rev_and_push.x[1] = (float)DJI_Rev_Motor.Data.Velocity_rads;
    rev_and_push.r_x[0] = rev_and_push.x[0];
    rev_and_push.r_x[1] = 0.0f;

#elif IS_HERO
		PID_Init(&pid_push_pos, PID_POSITION, pid_push_pos_params);
    PID_Init(&pid_push_vel, PID_POSITION, pid_push_vel_params);
	
    rev_and_push.x[0]   = DM_Rev_Motor.Data.Position;
    rev_and_push.r_x[1] = rev_and_push.x[0]; /* slew starts at current pos */
#endif
	is_first_iter = false;
}

  /*************************/
 /*   MAIN CONTROL LOOP   */
/*************************/

void control_loop_shooting(void){
		if (is_first_iter == true) _shooting_control_init();
	  #if !IS_SHOOTING_ENABLED
			shoot_wheels.ud[0] = 0;
			shoot_wheels.ud[1] = 0;
			rev_and_push.ud[0] = 0;
			rev_and_push.ud[1] = rev_and_push.x[0];
			return;
		#endif
    _control_loop_shoot_wheels();
    _control_loop_rev();
		#if IS_HERO
				_control_loop_push();
		#endif
	
}

  /*********************************/
 /*   SHOOT WHEELS CONTROL LOOP   */
/*********************************/

void _control_loop_shoot_wheels(void)
{
    if (state_remote_commands == COMMANDS_STOP) {
        shoot_wheels.ud[0] = 0;
        shoot_wheels.ud[1] = 0;
        pid_shoot_wheel_left.PID_Calc_Clear(&pid_shoot_wheel_left);
        pid_shoot_wheel_right.PID_Calc_Clear(&pid_shoot_wheel_right);
        return;
    }

    /* Update state */
    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels.x_prev[i]   = shoot_wheels.x[i];
        shoot_wheels.r_x_prev[i] = shoot_wheels.r_x[i];
    }
    shoot_wheels.x[0] = (float)DJI_Shooting_Motor[0].Data.Velocity_rads;
    shoot_wheels.x[1] = (float)DJI_Shooting_Motor[1].Data.Velocity_rads;

    /* Setpoint */
    switch (state_shoot_wheels) {
        case SHOOT_WHEELS_SPIN:
            shoot_wheels.r_x[0] = +r_shoot_wheels_ang_vel;
            shoot_wheels.r_x[1] = -r_shoot_wheels_ang_vel;
            break;
        case SHOOT_WHEELS_STOP:
        default:
            shoot_wheels.r_x[0] = 0.0f;
            shoot_wheels.r_x[1] = 0.0f;
            break;
    }

    /* Control */
    shoot_wheels.u[0] = PID_Calculate(&pid_shoot_wheel_left, shoot_wheels.r_x[0], shoot_wheels.x[0]);
    shoot_wheels.u[1] = PID_Calculate(&pid_shoot_wheel_right, shoot_wheels.r_x[1], shoot_wheels.x[1]);

    for (uint8_t i = 0; i < 2; i++) {
        shoot_wheels.ud[i] = shoot_wheels.u[i] * DJI_Motor_ADC[DJI_M3508];
    }
}

#if IS_STD || IS_SENTRY

        /************************************/
			 /*   _____  _______  _____          */
			/*   / ____||__   __||  __ \        */
		 /*		| (___     | |   | |  | |      */
		/*     \___ \    | |   | |  | |     */
	 /*		    ___) |   | |   | |__| |    */
	/*		   |_____/   |_|   |_____/    */
 /*	                                 */
/************************************/ 		

void _control_loop_rev(void)
{
    /* Update state */
    for (uint8_t i = 0; i < 2; i++) {
        rev_and_push.x_prev[i]   = rev_and_push.x[i];
        rev_and_push.r_x_prev[i] = rev_and_push.r_x[i];
    }
    rev_and_push.x[0] = (float)DJI_Rev_Motor.Data.Angle_sum;
    rev_and_push.x[1] = (float)DJI_Rev_Motor.Data.Velocity_rads;

    if (state_remote_commands == COMMANDS_STOP) {
        rev_and_push.ud[0] = 0;
			  rev_and_push.r_x[0] = rev_and_push.x[0];
        pid_rev_pos.PID_Calc_Clear(&pid_rev_pos);
        pid_rev_vel.PID_Calc_Clear(&pid_rev_vel);
        return;
    }

    /* Setpoint generation */
    switch (state_rev) {

        case REV_UNSTUCK:
            if (!unstuck_rev_enabled) {
                rev_and_push.r_x[0] = rev_and_push.x[0] - (pi / 4);
                unstuck_rev_enabled = true;
            }
            break;

        case REV_STOP:
            rev_and_push.r_x[1] = 0.0f;
            need_to_set_rev_ang_pos_reference = true;
						unstuck_rev_enabled = false;
            break;

        case REV_SINGLE_SHOOTING:
            if (need_to_set_rev_ang_pos_reference) {
                if (rev_and_push.r_x[0] - rev_and_push.x[0] < pi/2) rev_and_push.r_x[0] += (2.0f * pi / 8.0f);
                need_to_set_rev_ang_pos_reference = false;
            }
						unstuck_rev_enabled = false;
            break;

        default:
            break;
    }

    /* Outer loop: position -> velocity setpoint */
    rev_and_push.r_x[1] = PID_Calculate(&pid_rev_pos, rev_and_push.r_x[0], rev_and_push.x[0]);

    /* Inner loop: velocity -> current */
    rev_and_push.u[0]  = PID_Calculate(&pid_rev_vel, rev_and_push.r_x[1], rev_and_push.x[1]);
    rev_and_push.ud[0] = rev_and_push.u[0] * DJI_Motor_ADC[DJI_M2006];
		saturate(&rev_and_push.ud[0], 9000);
}

#elif IS_HERO
        /*******************************************/
			 /*   _    _  ______  _____    ____         */
			/*   | |  | ||  ____||  __ \  / __ \       */
		 /*		 | |__| || |__   | |__) || |  | |     */
		/*     |  __  ||  __|  |  _  / | |  | |    */
	 /*		   | |  | || |____ | | \ \ | |__| |   */
	/*		   |_|  |_||______||_|  \_\ \____/   */
 /*	                                        */
/*******************************************/ 	

void _control_loop_rev(void)
{
    /* Update state */
    for (uint8_t i = 0; i < rev_and_push.p; i++) {
        rev_and_push.x_prev[i]   = rev_and_push.x[i];
        rev_and_push.r_x_prev[i] = rev_and_push.r_x[i];
    }
    rev_and_push.x[0] = (float)DM_Rev_Motor.Data.Position;
		
		if (state_remote_commands == COMMANDS_STOP) {
        return;
    }
		
		if (is_homing_rev == 1) {
			rev_and_push.r_x[0] -= pi / 5000.0f;
			rev_and_push.ud[1] = rev_and_push.r_x[0];
			if (fabs(DM_Rev_Motor.Data.Torque) > 3.0f){
				is_homing_rev = 0;
				rev_and_push.r_x[0] += pi/3.7f;
				rev_and_push.ud[1] = rev_and_push.r_x[0];
		  }
			return;
		}	
			
		switch (state_rev) {

        case REV_UNSTUCK:
            if (!unstuck_rev_enabled) {
                rev_and_push.r_x[0] += (6.0f * pi) / 7.0f;
                unstuck_rev_enabled = true;
            }
            break;

        case REV_STOP:
            need_to_set_rev_ang_pos_reference = true;
						unstuck_rev_enabled = false;
            break;

        case REV_SINGLE_SHOOTING:
            if (need_to_set_rev_ang_pos_reference) {
							if( fabs(rev_and_push.r_x[0] -(6.0f * pi) / 7.0f) >= 90 || is_on_reset){
								is_on_reset = 1;
								rev_and_push.r_x[0] = 0;
							} else {
								rev_and_push.r_x[0] -= (6.0f * pi) / 7.0f;
							}
            }
						unstuck_rev_enabled = false;
            break;

        default:
            break;
    }
		
		if(!is_on_reset){
			/* Slew rate limits the position reference to avoid current spikes */
			slewRateControl(&rev_and_push.ud[1], rev_and_push.r_x[0], vel_up_rev, vel_down_rev, 0.001f);
		} else {
			rev_and_push.ud[1] = 0;
		}
}

void _control_loop_push(void)
{
    /* Update state (slot [2] in rev_and_push) */
    rev_and_push.x_prev[2]   = rev_and_push.x[2];
    rev_and_push.r_x_prev[2] = rev_and_push.r_x[2];

    rev_and_push.x[2] = (float)DJI_Push_Motor.Data.Angle_sum;   /* cumulative angle */

    if (state_remote_commands == COMMANDS_STOP) {
        rev_and_push.ud[0] = 0;
        pid_push_pos.PID_Calc_Clear(&pid_push_pos);
        pid_push_vel.PID_Calc_Clear(&pid_push_vel);
        return;
    }

    /* Latch position reference on first iteration */
    if (is_first_iter) {
        rev_and_push.r_x[2] = rev_and_push.x[2];
    }

    /* Setpoint generation */
    switch (state_push) {

        case PUSH_STOP:
            need_to_set_push_ang_pos_reference = true;
            break;

        case PUSH_SINGLE_SHOOTING:
            if (need_to_set_push_ang_pos_reference && !unstuck_push_enabled) {
                rev_and_push.r_x[2] = rev_and_push.x[2] + (float)(2.0f * pi / 2.0f);
                need_to_set_push_ang_pos_reference = false;
            }
            break;

        default:
            break;
    }

    /* Outer loop: position → velocity setpoint */
    float vel_setpoint = PID_Calculate(&pid_push_pos, rev_and_push.r_x[2], rev_and_push.x[2]);

    /* Inner loop: velocity → current */
    float vel_actual = (float)DJI_Push_Motor.Data.Velocity_rads;
    rev_and_push.u[1]  = PID_Calculate(&pid_push_vel, vel_setpoint, vel_actual);
    rev_and_push.ud[0] = rev_and_push.u[1] * DJI_Motor_ADC[DJI_M2006];
}

#endif