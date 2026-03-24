/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : pid.c
  * @brief          : pid functions 
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : To be perfected
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CONTROLLER_PID_H
#define CONTROLLER_PID_H

/* Includes ------------------------------------------------------------------*/
#include "config.h"
#include "lpf.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Macro definition of VAL_LIMIT that restricts the value of the specified variable.
 * @param x:   the specified variable
 * @param min: the minimum value of the specified variable
 * @param max: the maximum value of the specified variable
 * @retval none
 */
#define VAL_LIMIT(x,min,max)  do{ \
                                    if ((x) > (max)) {(x) = (max);} \
                                    else if ((x) < (min)) {(x) = (min);} \
                                }while(0U)

/**
 * @brief Macro definition of the number of PID parameters
 */
#ifndef PID_PARAMETER_NUM 
#define PID_PARAMETER_NUM 7							
#endif

/* Exported types ------------------------------------------------------------*/
/**
 * @brief Enum containing the error status for the PID controller.
 */
typedef enum
{
    PID_ERROR_NONE   = 0x00U,   /*!< No error */
    PID_FAILED_INIT  = 0x01U,   /*!< Initialization failed */
    PID_CALC_NANINF  = 0x02U,   /*!< Not-a-number (NaN) or infinity was generated */
    PID_Status_NUM,
} PID_Status_e;

/**
 * @brief Enum containing the control type for the PID controller.
 */
typedef enum
{
    PID_Type_None = 0x00U,   /*!< No type selected */
    PID_POSITION  = 0x01U,   /*!< Position PID - output is an absolute value */
    PID_VELOCITY  = 0x02U,   /*!< Velocity PID - output is an incremental value */
    PID_TYPE_NUM,
} PID_Type_e;

/**
 * @brief Structure containing the error handler information for the PID controller.
 */
typedef struct
{
    uint16_t   ErrorCount;   /*!< Counter for consecutive error occurrences */
    PID_Status_e Status;     /*!< Current error status */
} PID_ErrorHandler_Typedef;

/**
 * @brief Structure containing the tuning parameters for the PID controller.
 */
typedef struct
{
    float KP;             // Proportional gain
    float KI;             // Integral gain
    float KD;             // Derivative gain
    float Alpha;          // First-order low-pass filter coefficient for the derivative term
    float Deadband;       // Deadband threshold - PID stops calculating when |error| is below this value
    float LimitIntegral;  // Saturation limit for the integral term (anti-windup) [Ampere]
    float LimitOutput;    // Saturation limit for the total output [Ampere]
} PID_Parameter_Typedef;

/**
 * @brief Structure containing the full state and configuration of the PID controller.
 */
typedef struct _PID_TypeDef
{
    PID_Type_e Type;    // PID type: Position mode or Velocity mode (Position mode is most common)

    float Target;       // Desired setpoint value
    float Measure;      // Measured (feedback) value

    float Err[3];       // Error history: Err[0]=current, Err[1]=previous, Err[2]=two steps ago
                        // Error = Target - Measure
    float Integral;     // Accumulated integral value (sum of errors over time)
    float Pout;         // Proportional output: KP * Err[0]
    float Iout;         // Integral output:     KI * Integral
    float Dout;         // Derivative output:   KD * (discrete derivative of error)
    float Output;       // Total output:        Pout + Iout + Dout

    LowPassFilter1p_Info_TypeDef Dout_LPF;  // First-order low-pass filter applied to the derivative term

    PID_Parameter_Typedef    Param;         // PID tuning parameters structure
    PID_ErrorHandler_Typedef ERRORHandler;  // PID error handler structure

    /**
     * @brief Function pointer to initialize the PID parameters.
     *        Loads the parameter array into the PID controller structure.
     * @param PID:   Pointer to the _PID_TypeDef structure holding PID state and config.
     * @param Param: Pointer to the array of PID parameters to load.
     * @retval PID status - indicates whether initialization succeeded.
     */
    PID_Status_e (*PID_Param_Init)(struct _PID_TypeDef *PID, float *Param);

    /**
     * @brief Function pointer to reset all PID calculation values to zero.
     * @param PID: Pointer to the _PID_TypeDef structure holding PID state and config.
     * @retval none.
     */
    void (*PID_Calc_Clear)(struct _PID_TypeDef *PID);

} PID_Info_TypeDef;


/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Initializes the PID controller with a given type and parameter array.
 */
extern void PID_Init(PID_Info_TypeDef *Pid, PID_Type_e type, float para[PID_PARAMETER_NUM]);

/**
 * @brief Runs one iteration of the PID controller and returns the output.
 */
extern float PID_Calculate(PID_Info_TypeDef *PID, float Target, float Measure);

#endif // CONTROLLER_PID_H