/**
 ******************************************************************************
 * @file           : CAN_Task.c
 * @brief          : CAN task
 * @author         : GrassFam Wang
 * @date           : 2025/1/22
 * @version        : v2.0
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "CAN_Task.h"
#include "Control_Task.h"
#include "INS_Task.h"
#include "motor.h"
#include "bsp_can.h"
#include "Remote_Control.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "DJI_Motor.h"
#include "state_machine.h"
#include "shooting_control.h"
#include "chassis_control.h"
#include "gimbal_control.h"

static uint8_t is_first_iter = 1;
static uint8_t is_init = 1;

extern controlled_system_t shoot_wheels_and_rev;
extern float test_angle;

/* Debug: frequency measurement */
volatile uint32_t can_task_iter_count = 0;
volatile float    can_task_freq_hz = 0.0f;
static uint32_t   can_task_freq_start_ms = 0;
static uint32_t   can_task_freq_iters = 0;

void CAN_Task(void const * argument)
{
    /* Keep the task loop at 1ms (1000Hz) so our split halves result in 500Hz */
    const TickType_t xPeriod = 1 / portTICK_PERIOD_MS;
    can_task_freq_start_ms = HAL_GetTick();
    
    /* Flag used to alternate which half of the messages gets sent */
    static uint8_t split_flag = 0;

    for(;;)
    {
        /* Debug: measure actual frequency */
        can_task_iter_count++;
        can_task_freq_iters++;

        uint32_t now = HAL_GetTick();
        uint32_t elapsed = now - can_task_freq_start_ms;
        if (elapsed >= 1000) {
            can_task_freq_hz = (float)can_task_freq_iters * 1000.0f / (float)elapsed;
            can_task_freq_iters = 0;
            can_task_freq_start_ms = now;
        }

        /* One-time init */
        if (is_init) {
            //DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Save_Zero_Position);
            osDelay(30);
            is_init = 0;
        }

        /* If stop command arrived, disable motors once */
        if (state_remote_commands == COMMANDS_STOP) {
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], CM_Motor_Disable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Disable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Disable);
            osDelay(30);

            is_first_iter = 1;
        }
        else if (is_first_iter == 1) {
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], CM_Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Enable);
            osDelay(30);
            DM_Motor_Command(&FDCAN2_TxFrame, &DM_Yaw_Motor, Motor_Enable);
            osDelay(30);
            CM_Motor_Command(&FDCAN2_TxFrame, &CM_Pitch_Motor, CM_Motor_Save_Zero_Position);
            osDelay(30);

            is_first_iter = 0;
        }

        /* --- TIME SLICING: Send half the messages at a time --- */
        if (split_flag == 0) {
            // First Half: Chassis 0, 1 and Yaw
            #if IS_GIMBAL_ENABLED
                DM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &DM_Yaw_Motor, 0, gimbal.u[0], 0, 1, 0);
            #endif
            #if IS_CHASSIS_ENABLED
                CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[0], 0, chassis.r_x[0], 0, MIT_kd, 0);
                CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[1], 0, chassis.r_x[1], 0, MIT_kd, 0);
            #endif
            
            split_flag = 1; // Toggle flag for the next ms
        } 
        else {
            // Second Half: Chassis 2, 3 and Pitch
            #if IS_GIMBAL_ENABLED
                CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Pitch_Motor, gimbal.r_x[1] * 25 / 45, 0, 20, 1, 0);
            #endif
            #if IS_CHASSIS_ENABLED
                CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[2], 0, chassis.r_x[2], 0, MIT_kd, 0);
                CM_Motor_CAN_TxMessage(&FDCAN2_TxFrame, &CM_Chassis_Motor[3], 0, chassis.r_x[3], 0, MIT_kd, 0);
            #endif
            
            split_flag = 0; // Toggle flag back
        }

        #if !IS_REV_ENABLED
            shoot_wheels_and_rev.ud[2] = 0;
        #endif

        #if !IS_SHOOT_WHEELS_ENABLED
            shoot_wheels_and_rev.ud[0] = 0;
            shoot_wheels_and_rev.ud[1] = 0;
        #endif

        vTaskDelay(xPeriod); // Wait 1ms
    }
}