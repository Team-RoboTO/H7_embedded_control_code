#include "lidar_lifter_control.h"
#include "PID.h"
#include "DJI_motor.h"
#include "state_machine.h"
#include "Remote_Control.h"
#include "control_utils.h"

  /*************************/
 /*   CONTROLLED SYSTEM   */
/*************************/

controlled_system_t lidar_lifter = {
    .n          = 1,			// Number of system states	
    .m          = 2,			// Number of system inputs
    .p          = 1,			// Number of system outputs
};

  /*******************/
 /*   CONTROLLERS   */
/*******************/

static PID_Info_TypeDef pid_ll_pos;
static PID_Info_TypeDef pid_ll_vel;

// LL position PID (outer loop): KP is overwritten at runtime per shooting mode
static float pid_ll_pos_params[PID_PARAMETER_NUM] = {5.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 20.0f}; // TODO: tune

// LL velocity PID (inner loop)
static float pid_ll_vel_params[PID_PARAMETER_NUM] = {1.0f,  0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 5.0f}; // TODO: tune

static uint8_t is_first_iter                       = true;

float lidar_home_position = -1;
float LIDAR_CURRENT_TRESHOLD = 1000;
float CALIBRATION_SPEED =  10; // rad/s
float SETPOINT_DISTANCE = 105.0f;

  /********************/
 /*   CONTROL LOOP   */
/********************/

void control_loop_lidar_lifter(void){
	
			if (is_first_iter == 1) {
			PID_Init(&pid_ll_pos,  PID_POSITION, pid_ll_pos_params);
			PID_Init(&pid_ll_vel, PID_POSITION, pid_ll_vel_params);
			}
			
					 // On STOP: zero output and reset PIDs
			if (state_remote_commands == COMMANDS_STOP) {
					lidar_lifter.ud[0] = 0;
					pid_ll_pos.PID_Calc_Clear(&pid_ll_pos);
					pid_ll_vel.PID_Calc_Clear(&pid_ll_vel);
					return;
			}
		
			lidar_lifter.x[0] = (float) DJI_Lidar_Motor.Data.Angle_sum;           // [rad]
			lidar_lifter.x[1] = (float) DJI_Lidar_Motor.Data.Velocity_rads;		  	// [rad/s]

				
			if( lidar_home_position < 0 ){
				if( abs(DJI_Lidar_Motor.Data.Current) > LIDAR_CURRENT_TRESHOLD ){
					lidar_home_position = DJI_Lidar_Motor.Data.Angle_sum;
					lidar_lifter.r_x[0] = lidar_home_position; 					// Set home as base position
										
				} else {
					lidar_lifter.r_x[0] += CALIBRATION_SPEED/1000;
				}
			}
			
			if( lidar_home_position > 0 ){
				switch (state_lidar_lifter) {
					case LIDAR_UP:
						lidar_lifter.r_x[0] = lidar_home_position - SETPOINT_DISTANCE;
						break;
						
					case LIDAR_DOWN:
						lidar_lifter.r_x[0] = lidar_home_position - 1;
						break;
				}
			}
			
			
			// --- OUTER LOOP: Position Control ---
			// Output is the desired LL velocity setpoint
			lidar_lifter.u[0] = PID_Calculate(&pid_ll_pos, lidar_lifter.r_x[0], lidar_lifter.x[0]);
			saturate(&lidar_lifter.u[0], 20);
	
			// --- INNER LOOP: Velocity Control ---
			// Output is the motor current command

			lidar_lifter.u[1] = PID_Calculate(&pid_ll_vel, lidar_lifter.u[0], lidar_lifter.x[1]);
			lidar_lifter.ud[0] = lidar_lifter.u[1]*DJI_Motor_ADC[DJI_M2006]; //M2006_ADC_CONVERTION;
	
		return;
	is_first_iter = 0;
}