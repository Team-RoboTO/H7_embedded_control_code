#ifndef CONTROL_UTIL_H
#define CONTROL_UTIL_H

#include <stdint.h>


#define pi 3.1415926535
#define RAD_TO_DEG (180/pi)  // radiants-to-degrees ratio
#define DEG_TO_RAD (pi/180)  // degrees-to-radiants ratio
#define MAX_POLYN_ORDER 10  // max order of a polynomial (arbitrary)
#define EPSILON 1e-12  // very small constant used for numerical stability (e.g., instead of A/B, you can calculate A/(B+EPSILON) to avoid problems in case B == 0)
#define Cos45 0.7071067812f

  /******************/
 /*   MATH UTILS   */
/******************/

#define SEC(millisec) ((float)millisec)*1e-3  // computes the seconds corresponding to "millisec" milliseconds

  /*****************/
 /*   FUNCTIONS   */
/*****************/

void saturate(float *value, float bound);
void saturate_in_range(float *value, float lb, float ub);
void slewRateControl(float *profiled_value, float target_value, float max_accel, float max_decel, float dt);

float min(float a, float b);
float max(float a, float b);
int is_in_range(float value, float bound1, float bound2);
float nearest_target_angle_from_start_angle(float target_angle, float start_angle);


#endif