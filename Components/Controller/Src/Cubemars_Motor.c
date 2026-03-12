#include "cubemars_motor.h"
#include <string.h>
#include "stdint.h"

/**
 * @file    cubemars.c
 * @brief   CubeMars AK40-10 — CAN protocol + motor control
 */

  /****************************/
 /*   INTERNAL HELPERS       */
/****************************/

static float uint_to_float(int X_int, float X_min, float X_max, int Bits){
	
    float span = X_max - X_min;
    float offset = X_min;
    return ((float)X_int)*span/((float)((1<<Bits)-1)) + offset;
}

static int float_to_uint(float x, float x_min, float x_max, int bits){
	
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}





