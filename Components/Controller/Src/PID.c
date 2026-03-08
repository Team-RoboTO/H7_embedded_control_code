/**
  ******************************************************************************
  * @file           : PID.c
  * @brief          : PID functions 
  * @author         : GrassFan Wang
  * @date           : 2024/12/29
  * @version        : v1.1
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
/* Includes ------------------------------------------------------------------*/

/**
 * @brief Initializes the PID parameters.
 * @Param PID:   Pointer to the PID_Info_TypeDef structure holding PID state and config.
 * @Param Param: Pointer to the array of PID parameters to load.
 * @retval PID status.
 */
static PID_Status_e PID_Param_Init(PID_Info_TypeDef *PID, float Param[PID_PARAMETER_NUM])
{
    // Check if PID type and parameter array are valid.
    // If either is invalid, return PID_FAILED_INIT (initialization failure).
    if(PID->Type == PID_Type_None || Param == NULL)
    {
        return PID_FAILED_INIT;
    }
    
    // Load PID gains from the parameter array
    PID->Param.KP    = Param[0];
    PID->Param.KI    = Param[1];
    PID->Param.KD    = Param[2];
    PID->Param.Alpha = Param[3];

    // Initialize the derivative low-pass filter only if Alpha is in the valid range (0, 1)
    if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f)
        LowPassFilter1p_Init(&PID->Dout_LPF, PID->Param.Alpha);

    PID->Param.Deadband      = Param[4];
    PID->Param.LimitIntegral = Param[5];
    PID->Param.LimitOutput   = Param[6];

    // Reset the error counter
    PID->ERRORHandler.ErrorCount = 0;

    // Return PID_ERROR_NONE to indicate successful initialization
    return PID_ERROR_NONE;
}
//------------------------------------------------------------------------------


/**
 * @brief Resets all PID calculated values and internal state to zero.
 * @Param PID: Pointer to the PID_Info_TypeDef structure holding PID state and config.
 * @retval none.
 */
static void PID_Calc_Clear(PID_Info_TypeDef *PID)
{
    // Clear all error history, integral, and output terms to zero
    memset(PID->Err, 0, sizeof(PID->Err));
    PID->Integral = 0;

    PID->Pout   = 0;
    PID->Iout   = 0;
    PID->Dout   = 0;
    PID->Output = 0;
}
//------------------------------------------------------------------------------


/**
 * @brief Initializes the full PID controller (type, function pointers, and parameters).
 * @Param PID:   Pointer to the PID_Info_TypeDef structure holding PID state and config.
 * @Param Type:  PID control type (Position or Velocity).
 * @Param Param: Pointer to the array of PID parameters to load.
 * @retval PID status.
 */
void PID_Init(PID_Info_TypeDef *PID, PID_Type_e Type, float Param[PID_PARAMETER_NUM])
{
    PID->Type = Type;

    // Bind internal function pointers
    PID->PID_Calc_Clear = PID_Calc_Clear;
    PID->PID_Param_Init = PID_Param_Init;

    // Clear state before loading parameters
    PID->PID_Calc_Clear(PID);

    // Load parameters and store the resulting init status
    PID->ERRORHandler.Status = PID->PID_Param_Init(PID, Param);
}
//------------------------------------------------------------------------------


/**
 * @brief Checks the PID output for numerical errors (NaN or Inf).
 * @Param PID: Pointer to the PID_Info_TypeDef structure holding PID state and config.
 * @retval none.
 */
static void PID_ErrorHandle(PID_Info_TypeDef *PID)
{
    /* Judge NAN/INF */
    if(isnan(PID->Output) == true || isinf(PID->Output) == true)
    {
        PID->ERRORHandler.Status = PID_CALC_NANINF;
    }
}
//------------------------------------------------------------------------------

/**
 * @brief  Main PID calculation function. Runs one control iteration.
 * @Param  *PID    Pointer to a PID_Info_TypeDef structure that contains
 *                 the configuration information for the specified PID.
 * @Param  Target  Desired setpoint for the PID controller.
 * @Param  Measure Current measured value (feedback).
 * @retval The PID output.
 */
float PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure)
{
    /* Check and update the PID error status */
    PID_ErrorHandle(PID);
    if(PID->ERRORHandler.Status != PID_ERROR_NONE)
    {
        // If a numerical error occurred, reset all values and return 0 (safe state)
        PID->PID_Calc_Clear(PID);
        return 0;
    }

    /* Store the target and measurement */
    PID->Target  = Target;
    PID->Measure = Measure;

    /* Shift error history and compute the new current error */
    PID->Err[2] = PID->Err[1];
    PID->Err[1] = PID->Err[0];
    PID->Err[0] = PID->Target - PID->Measure;

    /* Only compute output if error exceeds the deadband threshold */
    if(fabsf(PID->Err[0]) >= PID->Param.Deadband)
    {
        if(PID->Type == PID_POSITION)
        {
            /* --- POSITION PID --- */

            /* Accumulate integral only if KI is non-zero; otherwise clear it */
            if(PID->Param.KI != 0)
                PID->Integral += PID->Err[0];
            else
                PID->Integral = 0;

            /* Clamp integral to prevent windup */
            VAL_LIMIT(PID->Integral, -PID->Param.LimitIntegral, PID->Param.LimitIntegral);

            /* Compute P, I, D terms */
            PID->Pout = PID->Param.KP * PID->Err[0];
            PID->Iout = PID->Param.KI * PID->Integral;
            PID->Dout = PID->Param.KD * (PID->Err[0] - PID->Err[1]);  // discrete first derivative

            /* Apply low-pass filter to derivative term if Alpha is valid */
            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f)
            {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            /* Sum and clamp total output */
            PID->Output = PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }
        else if(PID->Type == PID_VELOCITY)
        {
            /* --- VELOCITY (INCREMENTAL) PID --- */

            /* Compute P, I, D terms using error differences (incremental form) */
            PID->Pout = PID->Param.KP * (PID->Err[0] - PID->Err[1]);                          // change in error
            PID->Iout = PID->Param.KI * (PID->Err[0]);                                         // current error
            PID->Dout = PID->Param.KD * (PID->Err[0] - 2.f*PID->Err[1] + PID->Err[2]);        // second derivative (acceleration)

            /* Apply low-pass filter to derivative term if Alpha is valid */
            if(PID->Param.Alpha > 0.f && PID->Param.Alpha < 1.f)
            {
                PID->Dout_LPF.Alpha = PID->Param.Alpha;
                PID->Dout = LowPassFilter1p_Update(&PID->Dout_LPF, PID->Dout);
            }

            /* Accumulate increment into output and clamp */
            PID->Output += PID->Pout + PID->Iout + PID->Dout;
            VAL_LIMIT(PID->Output, -PID->Param.LimitOutput, PID->Param.LimitOutput);
        }
    }

    return PID->Output;
}