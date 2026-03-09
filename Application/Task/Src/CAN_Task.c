/**
  ******************************************************************************
  * @file           : CAN_Task.c
  * @brief          : CAN task
  * @author         : GrassFam Wang
  * @date           : 2025/1/22
  * @version        : v1.1
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

/* USER CODE BEGIN Header_CAN_Task */
/**
* @brief Function implementing the StartCANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Task */


float velocity_desired = 5;
 void CAN_Task(void const * argument)
{

 
  TickType_t CAN_Task_SysTick = 0;
	MIT_motor_Command(&FDCAN2_TxFrame,&chassis_motor[0],Motor_Enable);
  osDelay(30);
	MIT_motor_Command(&FDCAN2_TxFrame,&chassis_motor[1],Motor_Enable);
  osDelay(30);
  MIT_motor_Command(&FDCAN2_TxFrame,&chassis_motor[2],Motor_Enable);
  osDelay(30);
	MIT_motor_Command(&FDCAN2_TxFrame,&chassis_motor[3],Motor_Enable);
  osDelay(30);
	MIT_motor_Command(&FDCAN2_TxFrame,&gimbal_motor[0],Motor_Enable);
  osDelay(30);
	MIT_motor_Command(&FDCAN2_TxFrame,&gimbal_motor[1],Motor_Enable);
  osDelay(30);
	for(;;)
  {
	
  CAN_Task_SysTick = osKernelSysTick();
		
	
//	 // CAN-FD	 float Postion, float Velocity, float KP, float KD, float Torque
//	 MIT_motor_CAN_TxMessage(&FDCAN2_TxFrame,&chassis_motor[0],0,velocity_desired,0,1,0);
//	 MIT_motor_CAN_TxMessage(&FDCAN2_TxFrame,&chassis_motor[1],0,5,0,1,0);
//   MIT_motor_CAN_TxMessage(&FDCAN2_TxFrame,&chassis_motor[2],0,0,0,0,0);	
//   MIT_motor_CAN_TxMessage(&FDCAN2_TxFrame,&chassis_motor[3],0,0,0,0,0);	
//		FDCAN1_TxFrame.Header.Identifier = 0x200;
//		//Control_Info.SendValue[0] = 2000;
//    FDCAN1_TxFrame.Data[0] = 0x07;
//		FDCAN1_TxFrame.Data[1] = 0xD0;
//		FDCAN1_TxFrame.Data[2] = (uint8_t)(Control_Info.SendValue[1] >> 8);
//		FDCAN1_TxFrame.Data[3] = (uint8_t)(Control_Info.SendValue[1]);
//		FDCAN1_TxFrame.Data[4] = (uint8_t)(Control_Info.SendValue[2] >> 8); // motor 3
//		FDCAN1_TxFrame.Data[5] = (uint8_t)(Control_Info.SendValue[2]);
//		FDCAN1_TxFrame.Data[6] = (uint8_t)(Control_Info.SendValue[3] >> 8); // motor 4
//		FDCAN1_TxFrame.Data[7] = (uint8_t)(Control_Info.SendValue[3]);

   USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
		
	 if(CAN_Task_SysTick % 2 == 0){
	 
	  // this block only executes every 2 ticks
    // 1000Hz / 2 = 500Hz
	 
	 }	
		osDelay(1);
  }
 
}


