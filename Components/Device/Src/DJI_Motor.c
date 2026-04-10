#include "dji_motor.h"
#include <string.h>
#include "math_utils.h"

/**
 * @file    dji_motor.c
 * @brief   DJI M3508, GM6020 and M2006 CAN protocol + motor control
 */

  /***************************/
 /*   PARAMETERS FUNCTIONS  */
/***************************/

/**
  * @brief  float loop constrain
  * @param  Input    the specified variables
  * @param  minValue minimum number of the specified variables
  * @param  maxValue maximum number of the specified variables
  * @retval variables
  */
float F_Loop_Constrain(float Input, float Min_Value, float Max_Value)
{
  if (Max_Value < Min_Value)
  {
    return Input;
  }
  
  float len = Max_Value - Min_Value;    

  if (Input > Max_Value)
  {
      do{
          Input -= len;
      }while (Input > Max_Value);
  }
  else if (Input < Min_Value)
  {
      do{
          Input += len;
      }while (Input < Min_Value);
  }
  return Input;
}


/**
  * @brief  transform the Encoder(0-8192) to anglesum
  * @param  *Info        pointer to a Motor_Data_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor reduction ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval anglesum
  */
float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Reduction_Ratio,uint16_t MAXEncoder){
		/* 
		 * Update the cumulative angular position.
		 * Assumption of the algorithm: between two consecutive samples, the motor's encoder cannot
		 * travel more than 75% of a full rotation revolution (2*pi).
		 */
		float delta_ang_pos_digital = (float)(Data->Encoder - Data->Last_Encoder);
		
		if (delta_ang_pos_digital < -0.75f * MAXEncoder) {
				delta_ang_pos_digital += MAXEncoder;
		}
		else if (delta_ang_pos_digital > 0.75f * MAXEncoder) {
				delta_ang_pos_digital -= MAXEncoder;
		}
		delta_ang_pos_digital *= 2*pi/(MAXEncoder*Reduction_Ratio);
		Data->Angle_sum += delta_ang_pos_digital;
		
	 return Data->Angle_sum;
}


/**
  * @brief  transform the angular velocity [RPM] on the encoder shaft to the angular velocity [rad/s] of the output shaft
  * @param  *Info        pointer to a Motor_Data_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  Reduction_Ratio the specified motor reduction ratio
  * @retval velocity_rads
  */
float DJI_Motor_rpm_to_rads(DJI_Motor_Data_Typedef *Data, float Reduction_Ratio) {
    Data->Velocity_rads = Data->Velocity_rpm *2*pi/(60*Reduction_Ratio);
    return Data->Velocity_rads;
}

/**
  * @brief  transform the Encoder(0-8192) to angle(-180-180)
  * @param  *Data        pointer to a Motor_Data_Typedef structure that 
	*					             contains the Data for the specified motor
  * @param  torque_ratio the specified motor reduction ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval angle
  */
float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,float reduction_ratio,uint16_t MAXEncoder)
{	
  float Encoder_Err = 0.f;
  
  /* check the motor init */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = Data->Encoder/(MAXEncoder*reduction_ratio)*360.f;

    /* config the init flag */
    Data->Initlized = true;
  }
  
  Encoder_Err = Data->Encoder - Data->Last_Encoder;
  
  /* 0 -> MAXEncoder */		
  if(Encoder_Err > MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*reduction_ratio)*360.f;
  }
  /* MAXEncoder-> 0 */		
  else if(Encoder_Err < -MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*reduction_ratio)*360.f;
  }
  else
  {
    Data->Angle += (float)(Encoder_Err)/(MAXEncoder*reduction_ratio)*360.f;
  }
  
  /* loop constrain */
  Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);

  return Data->Angle;
}

  /*****************************/
 /*   COMUNICATION FUNCTIONS  */
/*****************************/

//------------------------------------------------------------------------------

/**
  * @brief  Transmit current setpoints to M3508 / M2006 motors (IDs 1-4).
  *         All four values are packed into a single 8-byte frame at ID 0x200.
  *         Current range: -16384 to +16384 (maps to ~-20A / +20A on M3508).
  * @param  FDCAN_TxFrame  pointer to the FDCAN TX frame to use (e.g. &FDCAN1_TxFrame)
  * @param  cur1..cur4     current setpoints for motor IDs 1-4
  * @retval None
  */
void DJI_M3508_M2006_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,
                                int16_t cur1, int16_t cur2,
                                int16_t cur3, int16_t cur4)
{
    FDCAN_TxFrame->Header.Identifier = 0x200;

    FDCAN_TxFrame->Data[0] = (uint8_t)(cur1 >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)(cur1);
    FDCAN_TxFrame->Data[2] = (uint8_t)(cur2 >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)(cur2);
    FDCAN_TxFrame->Data[4] = (uint8_t)(cur3 >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)(cur3);
    FDCAN_TxFrame->Data[6] = (uint8_t)(cur4 >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)(cur4);

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

/**
  * @brief  Transmit voltage setpoints to GM6020 motors.
  *         IDs 1-4  -> frame ID 0x1FF  (bytes 0-7)
  *         IDs 5-7  -> frame ID 0x2FF  (bytes 0-5, bytes 6-7 unused)
  *         Voltage range: -30000 to +30000.
  * @param  FDCAN_TxFrame  pointer to the FDCAN TX frame to use
  * @param  tx_id          CAN TX identifier: 0x1FF (motors 1-4) or 0x2FF (motors 5-7)
  * @param  vol1..vol4     voltage setpoints for the four slots in the frame
  *                        (slot 4 / vol4 unused when tx_id == 0x2FF)
  * @retval None
  */
void DJI_GM6020_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame, uint32_t tx_id,
                           int16_t vol1, int16_t vol2,
                           int16_t vol3, int16_t vol4)
{
    FDCAN_TxFrame->Header.Identifier = tx_id;

    FDCAN_TxFrame->Data[0] = (uint8_t)(vol1 >> 8);
    FDCAN_TxFrame->Data[1] = (uint8_t)(vol1);
    FDCAN_TxFrame->Data[2] = (uint8_t)(vol2 >> 8);
    FDCAN_TxFrame->Data[3] = (uint8_t)(vol2);
    FDCAN_TxFrame->Data[4] = (uint8_t)(vol3 >> 8);
    FDCAN_TxFrame->Data[5] = (uint8_t)(vol3);
    FDCAN_TxFrame->Data[6] = (uint8_t)(vol4 >> 8);
    FDCAN_TxFrame->Data[7] = (uint8_t)(vol4);

    USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);
}

//------------------------------------------------------------------------------

/**
  * @brief  Update the DJI motor Information
  * @param  Identifier  pointer to the specifies the standard identifier.
  * @param  Rx_Buf  pointer to the can receive data
  * @param  DJI_Motor pointer to a DJI_Motor_Info_t structure 
  *         that contains the information of DJI motor
  * @retval None
  */

void DJI_Motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* check the Identifier */
	if(*Identifier != DJI_Motor->FDCANFrame.RxIdentifier) return;
	
	/* update the last Encoder */
  DJI_Motor->Data.Last_Encoder =DJI_Motor->Data.Encoder;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = Rx_Buf[6];
	DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
	DJI_Motor->Data.Velocity_rpm = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
	DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);

	/* transform the Encoder to angle */
	switch(DJI_Motor->Type)
	{
		case DJI_GM6020:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192);
			DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,1.f,8192); 
			DJI_Motor->Data.Velocity_rads = DJI_Motor_rpm_to_rads(&DJI_Motor->Data,1.f); 
		break;
	
		case DJI_M3508:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,3591.f/187.f,8192);
			DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,3591.f/187.f,8192); 
			DJI_Motor->Data.Velocity_rads = DJI_Motor_rpm_to_rads(&DJI_Motor->Data,3591.f/187.f); 
		break;
		
		case DJI_M3508_SHOOTING_WHEELS:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,1.f,8192);
			DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,1.f,8192); 
			DJI_Motor->Data.Velocity_rads = DJI_Motor_rpm_to_rads(&DJI_Motor->Data,1.f); 
		break;
		
		case DJI_M2006:
			DJI_Motor->Data.Angle = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data,36.f,8192);
			DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data,36.f,8192); 
			DJI_Motor->Data.Velocity_rads = DJI_Motor_rpm_to_rads(&DJI_Motor->Data,36.f); 
		break;
		
		default:break;
	}
}