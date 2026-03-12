#include <math.h>
#include "math_utils.h"

  /*****************/
 /*   FUNCTIONS   */
/*****************/

float min(float a, float b) {
	
	return (a <= b) ? a : b;
}

float max(float a, float b) {
	
	return (a >= b) ? a : b;
}

int sign_zero_undef(float value){
	
	if (value > 0.0)
		return 1;
	else if (value < 0.0)
		return -1;
	else
		return 0;			/* NOTE: the number 0 has a non-specified sign (return 0) */
}

float max_abs(float *v, unsigned int size) {
	
	float max_abs = 0;
	
	for (unsigned int i = 0; i < size; i++) {
		
		if (fabs(v[i]) > max_abs)
			max_abs = fabs(v[i]);
	}
	
	return max_abs;
}

int is_in_range(float value, float bound1, float bound2) {
	
	float lb = min(bound1, bound2);		// get lower bound
	float ub = max(bound1, bound2);		// get upper bound
	
	/* return 1 if value is between lower and upper bound, 0 otherwise */
	if (lb <= value && value <= ub)
		return 1;
	
	return 0;
}

/*
void roots2coeff(int degree, float complex *roots, float *coeff) {

    double complex cmpx_coeff[degree + 1];

    // set initial coefficients to 0
    for (int i = 0; i <= degree; i++) {
        cmpx_coeff[i] = 0;
    }

    // compute the polynomial coefficients
    cmpx_coeff[degree] = 1;
    for (int i = 0; i < degree; i++) {
        for (int j = degree; j >= 1; j--) {
            cmpx_coeff[degree - j] = cmpx_coeff[degree - j + 1] - (roots[i]) * cmpx_coeff[degree - j];
        }
        cmpx_coeff[degree] *= -roots[i];
    }

    // return just the real part of coefficients (imaginary part is null because the only complex roots are conjugates, hence they cancel each other)
    for (int i = 0; i <= degree; i++) {
        coeff[i] = creal(cmpx_coeff[i]);
    }
}
*/

void mat_mult(float *result, float *mat1, float *mat2, uint8_t m, uint8_t p, uint8_t n) {

    for (int i = 0; i < m; i++) {
        for (int j = 0; j < n; j++) {

            result[i*n + j] = 0;
            for (int k = 0; k < p; k++) {
                result[i*n + j] += mat1[i*p + k] * mat2[k*n + j];
            }
        }
    }
}

float range_lin_prop(float in, float in_start, float in_end, float out_start, float out_end) {

    /* handle special cases */
    if (out_start == out_end)
        return out_start;                   // return the only possible output
    if (in_start == in_end)
        return (out_start + out_end) / 2;   // return the middle way between two output bounds

    /* regular case: "in_start != in_end" and "out_start != out_end" */
    float in_lb = min(in_start, in_end);    // input lower bound
    float in_ub = max(in_start, in_end);    // input upper bound

    // saturate input within the given input range
    if (in < in_lb)
        in = in_lb;
    if (in > in_ub)
        in = in_ub;

    float range_percent = fabs(in - in_start) / fabs(in_end - in_start);    // % of convered input range
    float out_offset = fabs(out_end - out_start) * range_percent;           // output offset related to the % of convered input range, starting from the beginning of the output range 
    float out = out_start;      // set output starting point, from which it will increase/decrease
    out_offset *= (out_start < out_end) ? +1 : -1;
    out += out_offset;

    return out;
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
