#include "power_estimation.h"

#include "controlled_system.h"
#include "arm_math.h"
#include "robot_config.h"
#include "motor.h"

/**
 * @brief Power model coefficients for different chassis types
 * The power model is: P = k1*τ² + τ*ω + k2*ω² + a = aτ² + bτ + c = eq. of grade 2 
 * Where:
 * - τ*ω: useful mechanical power (torque × angular velocity)
 * - k1*τ²: losses proportional to the square of the torque (winding losses)
 * - k2*ω²: losses proportional to the square of the velocity (friction/ventilation losses)
 * - a: constant losses (iron losses, constant friction)
 */

#if IS_STD
	float k1 = 338.2128;      // Coefficient for losses due to square of torque [W/Nm²]
	float k2 = 1.3252e-05;    // Coefficient for losses due to square of velocity [W·s²/rad²] (higher for sentry)
	float p0 = 4.081f;        // Constant losses [W]
#elif IS_SENTRY
	float k1 = 338.2128;      // Coefficient for losses due to square of torque [W/Nm²]
	float k2 = 1.3252e-03;    // Coefficient for losses due to square of velocity [W·s²/rad²] (higher for sentry)
	float p0 = 4.081f;        // Constant losses [W]
#elif IS_HERO
	float k1 = 338.2128;      // Coefficient for losses due to square of torque [W/Nm²]
	float k2 = 1.3252e-05;    // Coefficient for losses due to square of velocity [W·s²/rad²]
	float p0 = 4.081f;        // Constant losses [W]
#endif


// Motor torque conversion coefficient: (gear_ratio) * (Kt_motor)
// gear_ratio = ratio between motor revolutions and output shaft revolutions ≈ 187/3591 ≈ 0.052
// Kt_motor ≈ 0.3 Nm/A
float torque_coefficient = 1.56223893e-2f; // [Nm/A] conversion from current to torque at the output shaft
uint32_t t = 0;


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
 
void chassis_power_control(uint16_t limit, float *u){
	
	float chassis_power_limit = limit;  // [W] - Maximum power limit from the referee system
	
	// Arrays for calculating each motor's power
	float estimated_torque[4];        // [Nm] - Torque at the output shaft for each motor
	float estimated_give_power[4];    // [W]  - Estimated power for each motor
	float estimated_total_power = 0;  // [W]  - Estimated total power
	float scaled_give_power[4];       // [W]  - Scaled power for each motor after limiting
	float power_scale_factor = 0;            //  Power limiting scale factor

		/************************/
	 /*   POWER ESTIMATION   */
	/************************/
	
	for(int8_t i = 0; i < 4 ; i++ ){
		
		// Calculation of output shaft torque from current input
		estimated_torque[i] = u[i] * torque_coefficient;  // [Nm] = [A] * [Nm/A]
		
		// P = Mechanical Power + Torque Losses + Velocity Losses + Constant Losses
		estimated_give_power[i] = 
			estimated_torque[i] * chassis_motor[i].Data.Velocity/9.55                               // Mechanical Power: P_mech = τ*ω [W]
				+ k2 * chassis_motor[i].Data.Velocity * chassis_motor[i].Data.Velocity/9.55/9.55  // Velocity Losses: k2*ω² [W]
        + k1 * estimated_torque[i] * estimated_torque[i]                                          // Torque Losses: k1*τ² [W]
        + p0 ;                                                                                    // Constant Losses [W]
		
		if ( estimated_give_power[i] < 0) {  
			// If power is negative, the motor is acting as a generator
			// It does not contribute to the power limit, so we ignore this contribution
			continue;
		} else {
			// Sum only positive contributions (motors consuming power)
			estimated_total_power += estimated_give_power[i];  // [W]
		}
	}
	
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
		for (uint8_t i = 0; i < 4; i++) {
			scaled_give_power[i] = estimated_give_power[i] * power_scale_factor;
			if (scaled_give_power[i] < 0) {
				// Negative power: motor is regenerating, do not limit
				continue;
			}
			// Coefficients of the quadratic equation normalized by k1
      float b = chassis_motor[i].Data.Velocity/9.55/k1;
			float c = k2 * chassis_motor[i].Data.Velocity * chassis_motor[i].Data.Velocity/(k1*9.55*9.55) - scaled_give_power[i]/k1 + p0/k1;
			float delta = b * b - 4 * c;

			// No real solution: impossible to reach the target power --> maintain the current value
			if (delta < 0) {
				t += 1;
				continue; 
			}
			float new_output; //new output [A]
			if (u[i] > 0) {  
				// Positive torque: choose the positive root
				new_output = (-b + sqrt(delta)) / 2  / torque_coefficient ;
				u[i] = new_output;
			}
			else {
				// Negative torque: choose the negative root
				new_output = (-b - sqrt(delta)) / 2  / torque_coefficient;
				u[i] = new_output;
				
			}
		}
	}
}