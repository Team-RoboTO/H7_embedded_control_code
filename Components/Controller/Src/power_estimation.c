#include "power_estimation.h"

#include "controlled_system.h"
#include "arm_math.h"
#include "robot_config.h"
#include "cubemars_motor.h"
#include "rtt_log.h"
#include "segger_rtt.h"
#include "type_c_can.h"

#include "INA228.h"
#include "type_c_can.h"

#include "usart.h"
#include <stdio.h>
#include <string.h>

/**
 * Chiama questa funzione periodicamente
 * @param ts_ms      HAL_GetTick()
 * @param power_est  potenza stimata (W)
 * @param power_meas potenza misurata dal sensore (W)
 */
void Power_SendData(uint32_t ts_ms, float power_est, float power_meas)
{
    char buf[64];
    uint16_t len = (uint16_t)snprintf(buf, sizeof(buf),
        "$TS:%lu,PEST:%.3f,PMEAS:%.3f\n",
        (unsigned long)ts_ms,
        power_est,
        power_meas
    );

    /* Bloccante con timeout 10ms — USART10, nessun DMA */
    HAL_UART_Transmit(&huart10, (uint8_t*)buf, len, 10);
}

// Costanti fisiche ricavate dal datasheet AK40-10
float KT_OUT = 0.056f * 10.0f; // KT * Rapporto di riduzione = 0.56
float K1_JOULE = 0.733f;       // Perdite nel rame (0.75 * R_phase_to_phase)
float K2_IRON = 0.0094727f;        // Da calcolare empiricamente con i log di gara
float P0_STATIC = 1.8125f;        // Consumo in standby del driver (Watt)
 // FATTORE DI CORREZIONE: adatta la potenza elettrica interna (FOC) a quella reale (DC Bus)
const float CHASSIS_POWER_SCALE = 0.40692f;

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
<<<<<<< Updated upstream
=======

float estimated_total_power = 0;  // [W]  - Estimated total power
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
	
	estimated_total_power = 0;  // [W]  - Estimated total power
=======
	float estimated_give_power[4];    // [W]  - Estimated power for each motor
	estimated_total_power = 0;
	float scaled_give_power[4];       // [W]  - Scaled power for each motor after limiting
	float power_scale_factor = 0;            //  Power limiting scale factor
>>>>>>> Stashed changes

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
<<<<<<< Updated upstream

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
=======
		float chassis_power_limit = limit;  
	float estimated_give_power[4];    
	float scaled_give_power[4];       
	float power_scale_factor = 0;     

	// Azzera sempre l'accumulatore globale ad ogni ciclo
	estimated_total_power = 0.0f; 
	
	/************************/
	/* POWER ESTIMATION  */
	/************************/
	float sum_p_mech_raw = 0;
float sum_p_joule_raw = 0;
float sum_omega_sq = 0;
		
	for(int8_t i = 0; i < 4 ; i++ ){
		float current = CM_Chassis_Motor[i].Data.Torque;     
		float velocity = CM_Chassis_Motor[i].Data.Velocity;   
>>>>>>> Stashed changes
		
		// 1. Potenza Meccanica Teorica
		float p_mech = (current * KT_OUT) * velocity;
		
		// 2. Perdite Joule Teoriche
		float p_joule = K1_JOULE * current * current;
		
		sum_p_mech_raw += (current * KT_OUT) * velocity;
    sum_p_joule_raw += K1_JOULE * current * current;
    sum_omega_sq += velocity * velocity;
		
		
		// Calcolo della potenza grezza del singolo motore
		estimated_give_power[i] = (p_mech + p_joule)*CHASSIS_POWER_SCALE + P0_STATIC;
		
		
		if (estimated_give_power[i] < 0) {  
			continue;
		} else {
			estimated_total_power += estimated_give_power[i];  
		}
	}
<<<<<<< Updated upstream
	
	
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
	
=======
	    uint32_t ts   = HAL_GetTick();
    float p_est   = estimated_total_power;
    float p_meas  = Type_C_Can.value1;

    Power_SendData(ts, p_est, p_meas);

	values[1] = sum_p_mech_raw;
	values[2] = sum_p_joule_raw;
	values[3] = sum_omega_sq;
	values[0] = Type_C_Can.value1;
	RTT_Log(values, 4);
}
>>>>>>> Stashed changes
		/***************************/
	 /*   POWER LIMIT CONTROL   */
	/***************************/
	
//	if (estimated_total_power > chassis_power_limit) {
//		// Calculate the scaling factor to respect the power limit
//		//power_scale_factor = chassis_power_limit / estimated_total_power;
//		power_scale_factor = 1;
//		
//		/**
//	  * RECALCULATION OF CURRENTS FOR EACH MOTOR
//	  * For each motor consuming power, solves the equation:
//	  * P_scaled = k1*τ² + ω*τ + k2*ω² + p0
//	  * * Quadratic equation: A*τ² + B*τ + C = 0
//	  */
//		// wT +p0+ scaled power = 0     w = (scaled_power +po)/T
////		for (uint8_t i = 0; i < 4; i++) {
////			scaled_give_power[i] = estimated_give_power[i] * power_scale_factor;
////			if (scaled_give_power[i] < 0) {
////				// Negative power: motor is regenerating, do not limit
////				continue;
////			}
////			
//			//r_x[i] = (scaled_give_power[i] + p0)/CM_Chassis_Motor[i].Data.Torque;
//			
////			// Coefficients of the quadratic equation normalized by k1
////      float b = CM_Chassis_Motor[i].Data.Velocity;//k1;
////			float c = k2 * CM_Chassis_Motor[i].Data.Velocity * CM_Chassis_Motor[i].Data.Velocity/(k1*9.55*9.55) - scaled_give_power[i]/k1 + p0/k1;
////			float delta = b * b - 4 * c;

////			// No real solution: impossible to reach the target power --> maintain the current value
////			if (delta < 0) {
////				continue; 
////			}
////			float new_output; //new output [A]
////			if (r_x[i] > 0) {  
////				// Positive torque: choose the positive root
////				new_output = (-b + sqrt(delta)) / 2 ;
////				r_x[i] = new_output;
////			}
////			else {
////				// Negative torque: choose the negative root
////				new_output = (-b - sqrt(delta)) / 2 ;
////				r_x[i] = new_output;
////				
////			}
//		}
//	}
}