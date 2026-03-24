/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "Motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Control_Task.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "DJI_Motor.h"
#include "Remote_Control.h"
#include "state_machine.h"
#include "shooting_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"


/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */

static uint8_t is_first_iter = 1;

extern controlled_system_t shoot_wheels_and_rev;

extern float test_angle;

void CAN_Task(void const * argument)
{
    TickType_t CAN_Task_SysTick = 0;

    for(;;)
    {
		// If stop command arrived, send zeros as control signals
				if (state_remote_commands == COMMANDS_STOP) {
					// alternating logic to safely exit control mode for exit MIT control
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[0],CM_Motor_Disable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[1],CM_Motor_Disable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[2],CM_Motor_Disable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[3],CM_Motor_Disable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Disable);
					osDelay(30);
					DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Disable);
					osDelay(30);
					
					is_first_iter = 1;  // reset init flag
				}
				
				else if (is_first_iter == 1) {
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[0],CM_Motor_Enable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[1],CM_Motor_Enable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[2],CM_Motor_Enable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame,&CM_Chassis_Motor[3],CM_Motor_Enable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Enable);
					osDelay(30);
					DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Enable);
					osDelay(30);
					CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Save_Zero_Position);
					osDelay(30);
					is_first_iter = 0;
				}

				#if IS_GIMBAL_ENABLED
						DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_Yaw_Motor,0, 0,0,0,0);
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Pitch_Motor, test_angle * 25 / 45, 0, 20, 1, 0); 
						// pitch - MIT Mode
				//		CAN_Tx_MIT_Control(cm_p_des, cm_v_des, cm_kp, cm_kd, cm_t_ff, ID_pitch);

				//		// yaw - voltage mode
				//		CAN_Tx_gimbal((int16_t)gimbal.ud[0], 0);
				#endif
				
				#if IS_CHASSIS_ENABLED
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&CM_Chassis_Motor[0],0,chassis.r_x[0],0,0.2,0);
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&CM_Chassis_Motor[1],0,chassis.r_x[1],0,0.2,0);
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&CM_Chassis_Motor[2],0,chassis.r_x[2],0,0.2,0);	
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame,&CM_Chassis_Motor[3],0,chassis.r_x[3],0,0.2,0);	
				#endif
				
				#if !IS_REV_ENABLED
						shoot_wheels_and_rev.ud[2] = 0;
				#endif
				
				
				#if !IS_SHOOT_WHEELS_ENABLED
						shoot_wheels_and_rev.ud[0] = 0;
						shoot_wheels_and_rev.ud[1] = 0;
				#endif
				
        osDelay(1);
    }
}