#include "control_utils.h"
#include <math.h>

  /*****************/
 /*   FUNCTIONS   */
/*****************/

void saturate(float *value, float bound) {
	
	bound = fabs(bound);		// consider the absolute value of the given bound (a negative bound wouldn't make sense)
	
	if (*value > bound)
		*value = bound;
	else if (*value < -bound)
		*value = -bound;
}

void saturate_in_range(float *value, float lb, float ub) {
	
	if (*value < lb)
		*value = lb;
	if (*value > ub)
		*value = ub;
}

float dcgain(struct tf *tf) {

	float sum_num = 0;
	float sum_den = 0;
	uint8_t m = tf->m;
	uint8_t n = tf->n;

	for (int i = n-m; i <= n; i++)
			sum_num += tf->num[i];			// compute the sum of all tf numerator coefficients
	for (int i = 0; i <= n; i++)
			sum_den += tf->den[i];			// compute the sum of all tf denominator coefficients

	return sum_num/sum_den;					// return the tf steady-state gain
}

void set_dcgain(struct tf *tf, float new_dcgain) {

    float old_dcgain = dcgain(tf);		// compute the old tf steady-state gain
    uint8_t m = tf->m;
    uint8_t n = tf->n;

    for (int i = n-m; i <= n; i++)
        tf->num[i] *= (new_dcgain / old_dcgain);		// update the tf steady-state gain
}

void tf_init(struct tf *tf, float dt, uint8_t m, uint8_t n, float *num, float *den) {	
	
	tf->dt = dt;	// set tf sampling time
	tf->m = m;		// set tf numerator order
	tf->n = n;		// set tf denominator order
	
	/* set tf numerator/denominator and reset all tf inputs/outputs */
	for (int i = 0; i <= n; i++) {
		tf->num[i] = (i < n-m) ? 0 : num[i-n+m];
		tf->den[i] = den[i];
		tf->u[i] = 0;
    tf->y[i] = 0;
  }
}

void tf_reset_io(struct tf *tf) {

    for (int i = 0; i <= tf->m; i++) {
        tf->u[i] = 0;		// reset all tf inputs
        tf->y[i] = 0;		// reset all tf outputs
    }
}

float tf_resp(struct tf *tf, float u) {
	
	// reset the tf response to u input
	float y = 0;

	// shift backward u[k] and y[k] values of one time unit (for all k) 
	for (int i = tf->n; i > 0; i--) {
		tf->u[i] = tf->u[i-1];
		tf->y[i] = tf->y[i-1];
  }
  tf->u[0] = u;		// new input
	
	/*
	*		compute new output y[k] based on:
	*		1. new input u[k]
	*		2. previous inputs u[k-i] (i > 0)
	*		3. previous outputs y[k-i] (i > 0)
	*/
	for (int i = 0; i <= tf->n; i++) {
		y += tf->num[i] * tf->u[i];
    if (i > 0)
			y -= tf->den[i] * tf->y[i];
  }
	y = y / tf->den[0];

	tf->y[0] = y;		// new output

  return y;
}

void slewRateControl(float *profiled_value, float target_value, float max_accel, float max_decel, float dt) {
    float step  = target_value - *profiled_value; 
    float limit = ((target_value * *profiled_value < 0.0f) ||  
                    (fabsf(target_value) < fabsf(*profiled_value))) 
                   ? max_decel * dt
                   : max_accel * dt; 

	clamp(&step, -limit, limit); 

    *profiled_value += step;

	return;
}

void clamp(float *value, float lb, float ub) {
	if (*value < lb)
		*value = lb;
	if (*value > ub)
		*value = ub;
}