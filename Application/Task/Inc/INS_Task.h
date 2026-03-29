/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : INS_Task.h
  * @brief          : INS task
  * @author         : Yan Yuanbin
  * @date           : 2023/04/27
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INS_TASK_H
#define INS_TASK_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/* IMU config parameters -----------------------------------------------------*/
/**
 * @brief the flag of bmi088 Calibration
 *        0: DISABLE
 *        1: ENABLE
 */
#define IMU_Calibration_ENABLE  0U

/**
 * @brief the index of pitch angle update
 */
#define IMU_ANGLE_INDEX_PITCH  2U
/**
 * @brief the index of yaw angle update
 */
#define IMU_ANGLE_INDEX_YAW   0U
/**
 * @brief the index of roll angle update
 */
#define IMU_ANGLE_INDEX_ROLL   1U

/**
 * @brief the index of pitch gyro update
 */
#define IMU_GYRO_INDEX_PITCH  0U
/**
 * @brief the index of yaw gyro update
 */
#define IMU_GYRO_INDEX_YAW   2U
/**
 * @brief the index of roll gyro update
 */
#define IMU_GYRO_INDEX_ROLL   1U

/**
 * @brief the index of pitch accel update
 */
#define IMU_ACCEL_INDEX_PITCH  0U
/**
 * @brief the index of yaw accel update
 */
#define IMU_ACCEL_INDEX_YAW   2U
/**
 * @brief the index of roll accel update
 */
#define IMU_ACCEL_INDEX_ROLL   1U


/* Exported types ------------------------------------------------------------*/
/**
 * @brief typedef structure that contains the information for the INS.
 */
typedef struct 
{
	float Pitch_Angle;
	float Yaw_Angle;
	float Yaw_TolAngle;
	float Roll_Angle;

  float Pitch_Gyro;
  float Yaw_Gyro;
  float Roll_Gyro;

  float Angle[3];
	float Gyro[3];	
	float Accel[3];
	
	float Last_Yaw_Angle;
	int16_t YawRoundCount;

}INS_Info_Typedef;

/* Externs---------------------------------------------------------*/
extern INS_Info_Typedef INS_Info; 

#endif //INS_TASK_H
