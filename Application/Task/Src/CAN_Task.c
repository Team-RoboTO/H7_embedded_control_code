/**
 ******************************************************************************
 * @file           : CAN_Task.c
 * @brief          : CAN task
 * @author         : RoboTO
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "INS_Task.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "DJI_Motor.h"
#include "state_machine.h"
#include "shooting_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"
#include "Remote_Control.h"
#include "lidar_lifter_control.h"
#include "Referee_system.h"

static bool is_first_iter = 1;
static bool is_init = 1;

extern FDCAN_HandleTypeDef hfdcan2, hfdcan1;

void CAN_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
    
    /* Flag used to alternate which half of the messages gets sent */
    static uint8_t split_flag = 0;

		uint16_t prev_hp = 0;
	
    for(;;)
    {
				uint16_t current_hp = Referee_System_Info.robot_status.current_HP;
				
				bool reborn = (prev_hp == 0 && current_hp > 0);
			
			  /* Reset if CAN2 is dead*/
				if (hfdcan2.Instance->PSR & FDCAN_PSR_BO)
				{
						FDCAN2_Reset();
						osDelay(20);
						is_first_iter = 1;
						osDelay(20);
				}
				/* Reset if CAN2 is dead*/
				if (hfdcan1.Instance->PSR & FDCAN_PSR_BO)
				{
						FDCAN1_Reset();
						osDelay(20);
						is_first_iter = 1;
						osDelay(20);
				}
				
				/* One-time init */
        if (is_init) {
						#if IS_HERO
							DM_Motor_Command(&FDCAN3_TxFrame, &DM_Rev_Motor, Motor_Save_Zero_Position);
							osDelay(30);
						#endif
						CM_Motor_Command(&FDCAN1_TxFrame, &CM_Pitch_Motor, CM_Motor_Save_Zero_Position);
            osDelay(30);
            is_init = 0;
        }
				
        /* If stop command arrived, disable motors once */
        if (state_remote_commands == COMMANDS_STOP) {
						CM_Motor_Command(&FDCAN1_TxFrame, &CM_Pitch_Motor, CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], CM_Motor_Disable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Disable);
            osDelay(30);
						
						if (RC_info.RC.Left){
								DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Save_Zero_Position);
								osDelay(30);
						}
						
						#if IS_HERO
							DM_Motor_Command(&FDCAN3_TxFrame, &DM_Rev_Motor, Motor_Disable);
							osDelay(30);
						#endif
						
            is_first_iter = 1;
        }
        else if (is_first_iter == 1 || reborn) {
						if (reborn) osDelay(2200);  // wait for power after reborn
					
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], CM_Motor_Enable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Enable);
            osDelay(30);
						CM_Motor_Command(&FDCAN1_TxFrame, &CM_Pitch_Motor, CM_Motor_Enable);
						osDelay(30);
						#if IS_HERO
							DM_Motor_Command(&FDCAN3_TxFrame, &DM_Rev_Motor, Motor_Enable);
							osDelay(30);
						#endif
            is_first_iter = 0;
        }

        /* CAN2 TIME SLICING: Send half the messages at a time --- */
        if (split_flag == 0) {
            // First Half: Chassis 0, 1 and Yaw
						DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_Yaw_Motor, 0, gimbal.u[0], 0, KD_yaw, 0);
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], 0, chassis.r_x[0], 0, MIT_kd, 0);
						CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], 0, chassis.r_x[1], 0, MIT_kd, 0);
            
            split_flag = 1; // Toggle flag for the next ms
        } 
        else {
            // Second Half: Chassis 2, 3 and Pitch
            CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], 0, chassis.r_x[2], 0, MIT_kd, 0);
            CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], 0, chassis.r_x[3], 0, MIT_kd, 0);
            
            split_flag = 0; // Toggle flag back
        }
				
				/* CAN1 comunication */
				CM_Motor_CAN_TxMessage(&FDCAN1_TxFrame, &CM_Pitch_Motor, gimbal.r_x[1] * 25.0f / 45.0f, 0, KP_pitch, KD_pitch, 0);
				#if IS_STD || IS_SENTRY
				DJI_M3508_M2006_TxMessage(&FDCAN1_TxFrame, shoot_wheels.ud[0], shoot_wheels.ud[1], rev_and_push.ud[0], lidar_lifter.ud[0]);
				#elif IS_HERO
				DJI_M3508_M2006_TxMessage(&FDCAN1_TxFrame, shoot_wheels.ud[0], shoot_wheels.ud[1], rev_and_push.ud[0], 0);
				
				/* CAN3 comunication */
				if(is_on_reset){
					DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &DM_Rev_Motor, 0, 0 , 0, 0, 0);
					osDelay(1);
					DM_Motor_Command(&FDCAN3_TxFrame, &DM_Rev_Motor, Motor_Save_Zero_Position);
					osDelay(30);
					is_on_reset = 0;
				} else {
					DM_Motor_CAN_TxMessage(&FDCAN3_TxFrame, &DM_Rev_Motor, rev_and_push.ud[1], 0 , 10, 1, 0);
				}
								
				#endif
				
				prev_hp = current_hp;
				
				/* Motor disable check */
				
				float timestamp = HAL_GetTick();
				
				for(int i=0; i<4; i++){
					if(timestamp - CM_Chassis_Motor[i].Data.LastTimestamp > 500)
							CM_Chassis_Motor[i].Data.Error = 7;
				}
				for(int i=0; i<2;i++)
					if(timestamp - DJI_Shooting_Motor[i].Data.LastTimestamp > 500)
						DJI_Shooting_Motor[i].Data.Error = 1;
							
				
					else 
						DJI_Shooting_Motor[i].Data.Error = 0;
				if(timestamp - CM_Pitch_Motor.Data.LastTimestamp > 500)
					CM_Pitch_Motor.Data.Error = 7;
				#if IS_STD || IS_SENTRY
				if(timestamp - DJI_Rev_Motor.Data.LastTimestamp > 500)
					DJI_Rev_Motor.Data.Error = 1;
				else 
					DJI_Rev_Motor.Data.Error = 0;
				#elif IS_HERO
				if(timestamp - DJI_Push_Motor.Data.LastTimestamp > 500)
					DJI_Push_Motor.Data.Error = 1;
				else
					DJI_Push_Motor.Data.Error = 0;
				if(timestamp - DM_Rev_Motor.Data.LastTimestamp > 500)
					DM_Rev_Motor.Data.State = 7;
				
				#endif
				
				

				// Do the same thing for all the other motors.
			
        vTaskDelay(xPeriod); // Wait 1ms
    }
}

