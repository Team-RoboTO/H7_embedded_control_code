#include "power_estimation.h"

#include "controlled_system.h"
#include "arm_math.h"
#include "robot_config.h"
#include "cubemars_motor.h"
#include "rtt_log.h"
#include "segger_rtt.h"
#include "type_c_can.h"

#include "INA228.h"

#include "LPF.h"

LowPassFilter1p_Info_TypeDef Torque1_LPF1p;
LowPassFilter1p_Info_TypeDef Torque2_LPF1p;
LowPassFilter1p_Info_TypeDef Torque3_LPF1p;
LowPassFilter1p_Info_TypeDef Torque4_LPF1p;

// In power_estimation.c — aggiungi i buffer
static float torque_prev1[4] = {0};
static float torque_prev2[4] = {0};

static float median3(float a, float b, float c) {
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;
}

/**
 * @brief Power model coefficients for different chassis types
 * The power model is: P = k1*τ² + τ*ω + k2*ω² + a = aτ² + bτ + c = eq. of grade 2 
 * Where:
 * - τ*ω: useful mechanical power (torque × angular velocity)
 * - k1*τ²: losses proportional to the square of the torque (winding losses)
 * - k2*ω²: losses proportional to the square of the velocity (friction/ventilation losses)
 * - a: constant losses (iron losses, constant friction)
 */

float values[4];

float k1 = 0;//338.2128;      // Coefficient for losses due to square of torque [W/Nm²]
float k2 = 0;//1.3252e-05;    // Coefficient for losses due to square of velocity [W·s²/rad²] (higher for sentry)
float p0 = 1.85f;             // Constant losses [W]
float kt = 0.48f;

float estimated_give_power[4];    // [W]  - Estimated power for each motor
float estimated_total_power = 0;  // [W]  - Estimated total power
float scaled_give_power[4];       // [W]  - Scaled power for each motor after limiting
float power_scale_factor = 0;            //  Power limiting scale factor

bool is_first_iter = true;
/**
 * @brief Chassis power control algorithm
 * * This algorithm implements a power limiting system that:
 * 1. Estimates the required power for each motor using a mathematical model.
 * 2. If the total power exceeds the limit, it proportionally reduces the current (and thus the torque)
 * for all motors.
 * 3. Solves a quadratic equation to find the maximum admissible current.
 * * CURRENT STRATEGY: Current reduction (post-PID control)
 * ISSUES: 
 * - Can cause instability in the control system
 * - Not optimal for robot dynamics
 * - Reduces the effectiveness of the PID controller
 * * @param level: Chassis power limit in Watts [W]
 * * @param u: Array of 4 motor control currents [A]
 */
 
void chassis_power_control(uint16_t limit, float *r_x){
	
	float chassis_power_limit = limit;  // [W] - Maximum power limit from the referee system
	
	// Arrays for calculating each motor's power
	
	estimated_total_power = 0;  // [W]  - Estimated total power

	if(is_first_iter) {
		LowPassFilter1p_Init(&Torque1_LPF1p,0.96f);
		LowPassFilter1p_Init(&Torque2_LPF1p, 0.96f);
		LowPassFilter1p_Init(&Torque3_LPF1p, 0.96f);
		LowPassFilter1p_Init(&Torque4_LPF1p, 0.96f);
		
		is_first_iter = false;
	}
	
		/************************/
	 /*   POWER ESTIMATION   */
	/************************/
	
	for(int8_t i = 0; i < 4 ; i++ ){

    float raw = CM_Chassis_Motor[i].Data.Torque;
    
    // Step 1: mediana — elimina spike singoli
    float deglitched = median3(torque_prev2[i], torque_prev1[i], raw);
    torque_prev2[i] = torque_prev1[i];
    torque_prev1[i] = raw;
    
    // Step 2: LPF con Alpha = 0.90
    LowPassFilter1p_Info_TypeDef *lpf_array[4] = {
        &Torque1_LPF1p, &Torque2_LPF1p,
        &Torque3_LPF1p, &Torque4_LPF1p
    };

		// P = Mechanical Power + Torque Losses + Velocity Losses + Constant Losses
		estimated_give_power[i] = 
			CM_Chassis_Motor[i].Data.Torque *kt * CM_Chassis_Motor[i].Data.Velocity                                       // Mechanical Power: P_mech = τ*ω [W]
				+ k2 * r_x[i] * r_x[i]                                                       // Velocity Losses: k2*ω² [W]
        + k1 * CM_Chassis_Motor[i].Data.Torque * CM_Chassis_Motor[i].Data.Torque     // Torque Losses: k1*τ² [W]
        + p0 ;                                                                       // Constant Losses [W]
		
		if ( estimated_give_power[i] < 0) {  
			// If power is negative, the motor is acting as a generator
			// It does not contribute to the power limit, so we ignore this contribution
			continue;
		} else {
			// Sum only positive contributions (motors consuming power)
			estimated_total_power += estimated_give_power[i];  // [W]
		}
	}
	
	
//values[0] = CM_Chassis_Motor[0].Data.Torque;        // raw torque motore 1
//values[1] = LowPassFilter1p_Update(&Torque1_LPF1p,  // filtered (se non lo fai già nel loop)
//               CM_Chassis_Motor[0].Data.Torque);

values[0] = estimated_total_power;
values[1] = Type_C_Can.value1;
	
RTT_Log(values, 2);
	
//	values[0] = HAL_GetTick();
//	values[1] = estimated_total_power;
//	values[2] = INA228_ReadPower();
//	//values[2] = 0;
//	RTT_Log(values, 3);
	
		/***************************/
	 /*   POWER LIMIT CONTROL   */
	/***************************/
	
	if (estimated_total_power > chassis_power_limit) {
		// Calculate the scaling factor to respect the power limit
		power_scale_factor = chassis_power_limit / estimated_total_power;
		
		/**
	  * RECALCULATION OF CURRENTS FOR EACH MOTOR
	  * For each motor consuming power, solves the equation:
	  * P_scaled = k1*τ² + ω*τ + k2*ω² + p0
	  * * Quadratic equation: A*τ² + B*τ + C = 0
	  */
		// wT +p0+ scaled power = 0     w = (scaled_power +po)/T
		for (uint8_t i = 0; i < 4; i++) {
			scaled_give_power[i] = estimated_give_power[i] * power_scale_factor;
			if (scaled_give_power[i] < 0) {
				// Negative power: motor is regenerating, do not limit
				continue;
			}
			
			//r_x[i] = (scaled_give_power[i] + p0)/CM_Chassis_Motor[i].Data.Torque;
			
//			// Coefficients of the quadratic equation normalized by k1
//      float b = CM_Chassis_Motor[i].Data.Velocity;//k1;
//			float c = k2 * CM_Chassis_Motor[i].Data.Velocity * CM_Chassis_Motor[i].Data.Velocity/(k1*9.55*9.55) - scaled_give_power[i]/k1 + p0/k1;
//			float delta = b * b - 4 * c;

//			// No real solution: impossible to reach the target power --> maintain the current value
//			if (delta < 0) {
//				continue; 
//			}
//			float new_output; //new output [A]
//			if (r_x[i] > 0) {  
//				// Positive torque: choose the positive root
//				new_output = (-b + sqrt(delta)) / 2 ;
//				r_x[i] = new_output;
//			}
//			else {
//				// Negative torque: choose the negative root
//				new_output = (-b - sqrt(delta)) / 2 ;
//				r_x[i] = new_output;
//				
//			}
		}
	}
}