/**
  ******************************************************************************
  * @file           : Control_Task.c
  * @brief          : Control task
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.1
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Control_Task.h"
#include "cmsis_os.h"
#include "Control_Task.h"
#include "bsp_uart.h"
#include "Remote_Control.h"
#include "PID.h"
#include "Motor.h"
#include "arm_math.h"
#include "shooting_control.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "DJI_Motor.h"
#include "state_machine.h"
#include "gimbal_control.h"

static void Control_Init(Control_Info_Typedef *Control_Info);
static void Control_Measure_Update(Control_Info_Typedef *Control_Info);
static void Control_Target_Update(Control_Info_Typedef *Control_Info);
static void Control_Info_Update(Control_Info_Typedef *Control_Info);

Control_Info_Typedef Control_Info;
//                                  KP   KI   KD  Alpha Deadband  I_MAX   Output_MAX
static float Chassis_PID_Param[7] = {13.f,0.1f,0.f,0.f,  0.f,      5000.f,  12000.f};

PID_Info_TypeDef Chassis_PID;

void Control_Task(void const * argument)
{
  /* USER CODE BEGIN Control_Task */
  TickType_t Control_Task_SysTick = 0;
  
	Control_Init(&Control_Info);
 /* Infinite loop */
	for(;;)
  {
		Control_Task_SysTick = osKernelSysTick();
		robot_states_update_state_machine();
		control_loop_shooting();
		control_loop_gimbal();
	  Control_Measure_Update(&Control_Info);
		Control_Target_Update(&Control_Info);
    Control_Info_Update(&Control_Info);
    USART_Vofa_Justfloat_Transmit(Control_Info.Measure.Chassis_Velocity,0.f,0.f);
		
		osDelay(1);
  }
}
  /* USER CODE END Control_Task */

static void Control_Init(Control_Info_Typedef *Control_Info){

  PID_Init(&Chassis_PID,PID_POSITION,Chassis_PID_Param);

}

static void Control_Measure_Update(Control_Info_Typedef *Control_Info){

  Control_Info->Measure.Chassis_Velocity = DJI_Chassis_Motor[0].Data.Velocity_rpm;

}

static void Control_Target_Update(Control_Info_Typedef *Control_Info){

  Control_Info->Target.Chassis_Velocity = remote_ctrl.rc.ch[3] * 5.f;


}

static void Control_Info_Update(Control_Info_Typedef *Control_Info){
  
   PID_Calculate(&Chassis_PID, Control_Info->Target.Chassis_Velocity, Control_Info->Measure.Chassis_Velocity);

	 Control_Info->SendValue[0] = (int16_t)(Chassis_PID.Output);
	
}

static float FivePower(float NowTime,float UseTime){
     
	  float Time = (NowTime/UseTime);
	     
    return 10*powf(Time,3) - 15*powf(Time,4) + 6*powf(Time,5);


}
	