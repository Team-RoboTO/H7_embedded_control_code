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

void slewRateControl(float *profiled_value, float target_value, float max_accel, float max_decel, float dt) {
    float step  = target_value - *profiled_value; 
    float limit = ((target_value * *profiled_value < 0.0f) ||  
                    (fabsf(target_value) < fabsf(*profiled_value))) 
                   ? max_decel * dt
                   : max_accel * dt; 

	  saturate(&step, limit); 

    *profiled_value += step;

	return;
}

float min(float a, float b) {
	
	return (a <= b) ? a : b;
}

float max(float a, float b) {
	
	return (a >= b) ? a : b;
}

int is_in_range(float value, float bound1, float bound2) {
	
	float lb = min(bound1, bound2);		// get lower bound
	float ub = max(bound1, bound2);		// get upper bound
	
	/* return 1 if value is between lower and upper bound, 0 otherwise */
	if (lb <= value && value <= ub)
		return 1;
	
	return 0;
}

float nearest_target_angle_from_start_angle(float target_angle, float start_angle) {
	
	// NOTE: target_angle and start_angle can be whatever angle (unbounded)
	
	// if the two angles are identical, return the original target angle (no convertion needed)
	if (target_angle == start_angle)
		return target_angle;

	// count the number of complete rounds of target_angle
	uint32_t n_rounds_ref = (uint32_t) (fabs(target_angle) / (2*pi));
	
	// cast the target angle in range [-2*pi,+2*pi]
	target_angle += ((target_angle > 0) ? (-2*pi) : (2*pi)) * n_rounds_ref;

  // cast the target angle in range [-pi,+pi]
	if (target_angle > pi)
		target_angle -= 2*pi;
	else if (target_angle < -pi)
		target_angle += 2*pi;
	
	// count the number of complete rounds of start_angle
	uint32_t n_rounds_start = (uint32_t) (fabs(start_angle) / (2*pi));
	
	// bring target_angle in range [start_angle - 2*pi, start_angle + 2*pi]
	target_angle += ((target_angle > start_angle) ? (-2*pi) : (2*pi)) * n_rounds_start;
	
	// if target_angle is already at the minimum distance from start_angle, then return it
	if (fabs(target_angle - start_angle) <= pi)
		return target_angle;
	
	if (target_angle < start_angle)
		return target_angle + 2*pi;
	
	return target_angle - 2*pi;
}
