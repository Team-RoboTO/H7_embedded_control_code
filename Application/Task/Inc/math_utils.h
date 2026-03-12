#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <stdint.h>

  /**********************/
 /*   MATH CONSTANTS   */
/**********************/

#define pi 3.1415926535
#define RAD_TO_DEG (180/pi)  // radiants-to-degrees ratio
#define DEG_TO_RAD (pi/180)  // degrees-to-radiants ratio
#define MAX_POLYN_ORDER 10  // max order of a polynomial (arbitrary)
#define EPSILON 1e-12  // very small constant used for numerical stability (e.g., instead of A/B, you can calculate A/(B+EPSILON) to avoid problems in case B == 0)

  /******************/
 /*   MATH UTILS   */
/******************/

#define SEC(millisec) ((float)millisec)*1e-3  // computes the seconds corresponding to "millisec" milliseconds

  /*****************/
 /*   FUNCTIONS   */
/*****************/

/************************************************************************************************************************
	NAME: min

	DESCRIPTION: returns the minimum among its arguments
************************************************************************************************************************/
float min(float a, float b);

/************************************************************************************************************************
	NAME: max

	DESCRIPTION: returns the maximum among its arguments
************************************************************************************************************************/
float max(float a, float b);

/************************************************************************************************************************
	NAME: sign_zero_undef

	DESCRIPTION:
		- returns +1/-1 if the argument is positive/negative.
		- returns 0 if the argument is zero.
************************************************************************************************************************/
int sign_zero_undef(float value);

/************************************************************************************************************************
	NAME: max_abs

	DESCRIPTION: given an array, returns the max absolute value of all array's entries
	
	ARGUMENTS:
	- v:   		pointer to the array
	- size:   array size
************************************************************************************************************************/
float max_abs(float *v, unsigned int size);

/************************************************************************************************************************
	NAME: is_in_range

	DESCRIPTION: checks if a given value is inside a given range
	
	ARGUMENTS:
	- value:  	value to check
	- bound1:		1st bound of the range
	- bound2:		2nd bound of the range
	
	NOTE: it's not necessary that bound1 is lower than bound2 or vice versa.
************************************************************************************************************************/
int is_in_range(float value, float bound1, float bound2);

/************************************************************************************************************************
	NAME: roots2coeff

	DESCRIPTION: given the roots of a polynomial of a certain degree, computes the coefficients of the polynomial

	ARGUMENTS:
	- degree:   polynomial order
	- roots:    array of polynomial roots (can also be complex numbers)
	- coeff:    polynomial coefficients

	EXAMPLES:
	1.
	 (z - (-1+1i)) * (z - (-1-1i)) = (z^2 + 2*z + 2)

	if...
			degree = 2;
			roots = {-1+1i, -1-1i};
	then...
			coeff = {1, 2, 2};

	2.
	 (z + 2) * (z + 3) = (z^2 + 5*z + 6)

	if...
			degree = 2;
			roots = {-2, -3};
	then...
			coeff = {1, 5, 6};
************************************************************************************************************************/
//void roots2coeff(int degree, float complex *roots, float *coeff);

/************************************************************************************************************************
	NAME: mat_mult

	DESCRIPTION: computes the multiplication of 2 given matrices (having the given dimensions)

	ARGUMENTS:
	- result:   matrix computed as the multiplication of mat1 and mat2
	- mat1:    	left matrix to be multiplied
	- mat2:    	right matrix to be multiplied
	- m:    		number of rows of mat1
	- p:				number of columns of mat1, and number of rows of mat2 (mat1 and mat2 dimensions must be coherent)
	- n:				number of columns of mat2

	EXAMPLES:
	1.
		if...
			mat1 = {1 2 3 4 5 6};
			mat2 = {2 0 -1};
			m = 2;
			p = 3;
			n = 1;
		then...
			result = mat1 * mat2 = {1*2+2*0+3*(-1) 4*2+5*0+6*(-1)} = {-1 2};	<-- result is a "m x n" matrix (2 x 1)	
	
	NOTE: matrices always have to be represented as 1D arrays (you can manage it as matrices by knowing their number of rows and columns)
************************************************************************************************************************/
void mat_mult(float *result, float *mat1, float *mat2, uint8_t m, uint8_t p, uint8_t n);

/************************************************************************************************************************
	NAME: range_lin_prop
	
	DESCRIPTION:
  - given an input value in a certain input range, returns an output (inside the given output range) linearly proportional to the input
  - the linear proportion between input and output can be either direct (increase input ==> increase output, and vice versa) or inverse (increase input ==> decrease output, and vice versa).  
	- if input is outside of the given input range bounds ==> return one of the two output range bounds as output (the one on the same "side" as the given input)

	ARGUMENTS:
	- in:		    input value
	- in_start:     1st bound of input range
	- in_end:       2nd bound of input range
	- out_start:    1st bound of output range
	- out_end:      2nd bound of output range

	NOTE:
	- in_start could be lower of higher than in_end
	- out_start could be lower of higher than out_end
	- if input range size is 0 (i.e. there's only one value inside it)  ==> return the middle way between two output bounds
	- if output range size is 0 (i.e. there's only one value inside it) ==> return the only possible output

	EXAMPLES:
	1. range_lin_prop(3,2,4,7,9)    returns 8
	2. range_lin_prop(1.5,2,4,7,9)  returns 7
	3. range_lin_prop(8,2,4,7,9)    returns 9
	4. range_lin_prop(4,5,2,6,9)    returns 7
	5. range_lin_prop(4,5,2,9,6)    returns 8
	6. range_lin_prop(1,2,2,4,5)    returns 4.5
	7. range_lin_prop(1,10,4,7,7)   returns 7
************************************************************************************************************************/
float range_lin_prop(float in, float in_start, float in_end, float out_start, float out_end);

/************************************************************************************************************************
    NAME: nearest_target_angle_from_start_angle

    DESCRIPTION: given a (cumulative) target angle, converts it into the (cumulative) angle that is the nearest from the given starting angle.

    ARGUMENTS:
    - target_angle: target angle to be converted [rad]
    - start_angle: starting angle [rad]

    EXAMPLES: (NOTE that these examples are in degrees for bettere undestanding, but THE FUNCTION EXPECTS RADIANTS as inputs)
    1. nearest_target_angle_from_start_angle(10, 0)      returns 10
    2. nearest_target_angle_from_start_angle(-170, 170)  returns 190
    3. nearest_target_angle_from_start_angle(160, -130)  returns -200
    4. nearest_target_angle_from_start_angle(10, -680)   returns -710
    5. nearest_target_angle_from_start_angle(370, 0)     returns 10

    // NOTE: target_angle and start_angle can be whatever angle (unbounded)
************************************************************************************************************************/
float nearest_target_angle_from_start_angle(float target_angle, float start_angle);

#endif