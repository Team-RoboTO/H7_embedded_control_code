#ifndef CONTROL_UTIL_H
#define CONTROL_UTIL_H

#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include "math_utils.h"
#include "stdint.h"

  /********************************/
 /*   TRANSFER FUNCTION STRUCT   */
/********************************/

/************************************************************************************************************************
	NAME: tf
	
	DESCRIPTION: data structure that can contain a discrete transfer function

					b[m]*z^{m-n} + b[m-1]*z^{m-n-1} + ... + b[0]*z^{-n}
	TF = --------------------------------------------------------
			 a[n] + a[n-1]*z^{-1} + a[n-2]*z^{-2} + ... + a[0]*z^{-n}
		
	num = {b[n], b[n-1], ..., b[0]};		NOTE: here the index is intentionally "n", not "m"
	den = {a[n], a[n-1], ..., a[0]};

	y = {y[k], y[k-1], ..., y[k-n]};
	u = {u[k], u[k-1], ..., u[k-n]};

	EXAMPLES:
	1.
				5*z^2 - 9.75*z + 4.753
	 --------------------------------
	 z^3 - 2.9*z^2 + 2.803*z - 0.9029

	m = 2;
	n = 3;
	num = {0, 5, -9.75, 4.753};					NOTE: here num[0] = 0 (i.e. b[3] = 0) because m = 2, n = 3, n-m = 1 (there's a delay of one time unit)
	den = {1, -2.9, 2.803, -0.9029};
	
	NOTE: you must have "m <= n" in order to have a proper transfer function
************************************************************************************************************************/
struct tf {

    float 	dt;			// sample time of the discrete tf
    uint8_t m;			// order of tf numberator
    uint8_t n; 			// order of tf denominator
    float 	num[MAX_POLYN_ORDER + 1];  		// tf numberator coefficients
    float 	den[MAX_POLYN_ORDER + 1];    	// tf denominator coefficients
    float 	y[MAX_POLYN_ORDER + 1];      	// tf outputs
    float 	u[MAX_POLYN_ORDER + 1];      	// tf inputs
};

  /*****************/
 /*   FUNCTIONS   */
/*****************/

void saturate(float *value, float bound);
void saturate_in_range(float *value, float lb, float ub);

/************************************************************************************************************************
	NAME: dcgain
	
	DESCRIPTION: returns the steady-state gain of the given transfer function
************************************************************************************************************************/
float dcgain(struct tf *tf);

/************************************************************************************************************************
	NAME: set_dcgain
	
	DESCRIPTION: modifies the numerator of the given transfer function in order to set its steady-state gain to the desired value
	
	ARGUMENTS:
	- tf: 				pointer to the transfer function
	- new_dcgain:	new desired steady-state gain
************************************************************************************************************************/
void set_dcgain(struct tf *tf, float new_dcgain);

/************************************************************************************************************************
	NAME: tf_init
	
	DESCRIPTION: initialize a discrete transfer function

	ARGUMENTS:
	- tf:		pointer to the tf
	- dt:   tf sample time
	- m:    order of tf numerator
	- n:		order of tf denominator
	- num:	tf numerator
	- den:	tf denominator

	EXAMPLES:
	1.
	      5*z^2 - 9.75*z + 4.753
	 --------------------------------
	 z^3 - 2.9*z^2 + 2.803*z - 0.9029

		Inputs to function:
		dt = 0.01;
		m = 2;
		n = 3;
		num = {5, -9.75, 4.753};						NOTE: not {0, 5, -9.75, 4.753}
		den = {1, -2.9, 2.803, -0.9029};
		
		Outputs of function:
		tf->dt = 0.01;
		tf->m = 2;
		tf->n = 3;
		tf->num = {0, 5, -9.75, 4.753};			NOTE: now we have {0, 5, -9.75, 4.753}, where the initial 0 takes into account the delay of "n-m = 1" time units
		tf->den = {1, -2.9, 2.803, -0.9029};
************************************************************************************************************************/
void tf_init(struct tf *tf, float dt, uint8_t m, uint8_t n, float *num, float *den);

/************************************************************************************************************************
	NAME: tf_reset_io
	
	DESCRIPTION: sets to 0 all the IO (inputs and outputs) of the given transfer function
								
	y[k] = y[k-1] = ... = y[k-n] = 0;
	u[k-n+m] = u[k-n+m-1] = ... = u[k-n] = 0;
************************************************************************************************************************/
void tf_reset_io(struct tf *tf);

/************************************************************************************************************************
	NAME: tf_resp
	
	DESCRIPTION: computes the response of a transfer function to a given input

	ARGUMENTS:
	- tf:		pointer to the tf
	- u:		input at current time (i.e. u[k])

	The tf response y[k] is computed by:
	
	y[k] = (b[n]*u[k] + b[n-1]*u[k-1] + b[n-2]*u[k-2] + ... + b[0]*u[k-n] - ...
								... - a[n-1]*y[k-1] - b[n-2]*y[k-2] - ... - a[0]*y[k-n]) / a[n];
	
	
	NOTE: if m < n, then inputs are delayed by n-m time units ==> y[k] does NOT depend on u[k], u[k-1], ..., u[k-n+m+1]
	
					b[m]*z^{m-n} + b[m-1]*z^{m-n-1} + ... + b[0]*z^{-n}
	TF = --------------------------------------------------------
			 a[n] + a[n-1]*z^{-1} + a[n-2]*z^{-2} + ... + a[0]*z^{-n}
************************************************************************************************************************/
float tf_resp(struct tf *tf, float u);

/************************************************************************************************************************
    NAME: clamp
    
    DESCRIPTION: constraints a value within a specific numerical range [lb, ub]
************************************************************************************************************************/
void clamp(float *value, float lb, float ub);

/************************************************************************************************************************
    NAME: slewRateControl
    
    DESCRIPTION: computes the profiled response of a velocity command to prevent wheel slippage
                 by enforcing acceleration and deceleration limits

    ARGUMENTS:
    - profiled_value: pointer to the current state (i.e. v_profiled[k-1])
    - target_value:   desired input velocity (i.e. v_target[k])
    - max_accel:      maximum allowed acceleration [units/s^2]
    - max_decel:      maximum allowed deceleration [units/s^2]
    - dt:             delta time since last update [s]

    The profiled output v_profiled[k] is computed by:
    
    1. error = v_target[k] - v_profiled[k-1]
    2. limit = (decelerating or direction_change) ? (max_decel * dt) : (max_accel * dt)
    3. step  = sat(error, -limit, limit)
    4. v_profiled[k] = v_profiled[k-1] + step
    
    NOTE: deceleration is detected if |v_target| < |v_profiled| or if v_target * v_profiled < 0
************************************************************************************************************************/
void slewRateControl(float *profiled_value, float target_value, float max_accel, float max_decel, float dt);


#endif