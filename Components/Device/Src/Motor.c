/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v1.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "Motor.h"

/**
 * @brief The structure that contains the Information of yaw motor.Use DJI GM6020 motor.
 */
MIT_motor_Info_Typedef gimbal_motor[2] = 
{
	  [0] = {
			.Control_Mode = MIT,
			.Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
			},
		  .FDCANFrame = {
				 .TxIdentifier = 0x05,
				 .RxIdentifier = 0x15,
			}
		},
		
    [1] = {
			.Control_Mode = MIT,	
			.Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
				
			},	
		  .FDCANFrame = {
				 .TxIdentifier = 0x05,
				 .RxIdentifier = 0x15,
			}
		},
};
//------------------------------------------------------------------------------

/**
 * @brief The structure that contains the Information of chassis motor.Use DJI M3508 motor.
 */ 
DJI_Motor_Info_Typedef shooting_motor[3] = {

    [0] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x201,
				}
    },
    [1] = {	
        .Type = DJI_M3508,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x202,
				}
    },
	  [2] = {	
        .Type = DJI_M2006,
		    .FDCANFrame = {
					  .TxIdentifier = 0x200,
					  .RxIdentifier = 0x203,
				}
		},

};
//------------------------------------------------------------------------------

/**
 * @brief The structure that contains the Information of joint motor.Use DM 8009 motor.
 */
MIT_motor_Info_Typedef chassis_motor[4]= {
    
	  [0] = {
			.Control_Mode = MIT,
			.Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
			},
		  .FDCANFrame = {
				 .TxIdentifier = 0x11,
				 .RxIdentifier = 0x01,
			},
		},
		
    [1] = {
			.Control_Mode = MIT,	
			.Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
				
			},	
		  .FDCANFrame = {
				 .TxIdentifier = 0x02,
				 .RxIdentifier = 0x12,
			},
		},
		
    [2] = {
			.Control_Mode = MIT,
      .Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
				
			},	
		  .FDCANFrame = {
				 .TxIdentifier = 0x03,
				 .RxIdentifier = 0x13,
			},
		},
		
	  [3] = {
			 .Control_Mode = MIT,	
			 .Param_Range ={
			   .P_MAX = 3.141593f,
			   .V_MAX = 45.f,
			   .T_MAX = 54.f		
				
			},	
		  .FDCANFrame = {
				 .TxIdentifier = 0x04,
				 .RxIdentifier = 0x14,
			},
		},
  

};
//------------------------------------------------------------------------------

/**
  * @brief  编码器值转化为角度(累加 最到到float最大值)
  */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *,float ,uint16_t );
/**
  * @brief  编码器值转化为角度(范围 正负180度)
  */
static float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *,float ,uint16_t );

float F_Loop_Constrain(float Input, float Min_Value, float Max_Value);

static float uint_to_float(int X_int, float X_min, float X_max, int Bits);

static int float_to_uint(float x, float x_min, float x_max, int bits);

//------------------------------------------------------------------------------

/**
  * @brief  Update the DJI motor Information
  * @param  Identifier  pointer to the specifies the standard identifier.
  * @param  Rx_Buf  pointer to the can receive data
  * @param  DJI_Motor pointer to a DJI_Motor_Info_t structure 
  *         that contains the information of DJI motor
  * @retval None
  */
void DJI_motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf,DJI_Motor_Info_Typedef *DJI_Motor)
{
	/* check the Identifier */
	if(*Identifier != DJI_Motor->FDCANFrame.RxIdentifier) return;
	
	/* transforms the  general motor data */
	DJI_Motor->Data.Temperature = Rx_Buf[6];
	DJI_Motor->Data.Encoder  = ((int16_t)Rx_Buf[0] << 8 | (int16_t)Rx_Buf[1]);
	DJI_Motor->Data.Velocity = ((int16_t)Rx_Buf[2] << 8 | (int16_t)Rx_Buf[3]);
	DJI_Motor->Data.Current  = ((int16_t)Rx_Buf[4] << 8 | (int16_t)Rx_Buf[5]);

	/* transform the Encoder to angle */
	switch(DJI_Motor->Type){
    case DJI_GM6020:
        DJI_Motor->Data.Angle     = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data, 1.f, 8192);
        DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data, 1.f, 8192); // ? aggiungi
        break;

    case DJI_M3508:
        DJI_Motor->Data.Angle     = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data, 3591.f/187.f, 8192);
        DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data, 3591.f/187.f, 8192); // ? aggiungi
        break;

    case DJI_M2006:
        DJI_Motor->Data.Angle     = DJI_Motor_Encoder_To_Angle(&DJI_Motor->Data, 36.f, 8192);
        DJI_Motor->Data.Angle_sum = DJI_Motor_Encoder_To_Anglesum(&DJI_Motor->Data, 36.f, 8192); // ? aggiungi
        break;

    default: break;
	}
}
//------------------------------------------------------------------------------

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
//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to anglesum(3.4E38)
  * @param  *Info        pointer to a Motor_Data_Typedef structure that 
	*					             contains the infomation for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval anglesum
  */
static float DJI_Motor_Encoder_To_Anglesum(DJI_Motor_Data_Typedef *Data,float Torque_Ratio,uint16_t MAXEncoder)
{
  float res1 = 0,res2 =0;
  
  if(Data == NULL) return 0;
  
  /* Judge the motor Initlized */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle_sum = 0;

    /* Set the init flag */
    Data->Initlized = true;
  }
  
  /* get the possiable min Encoder err */
  if(Data->Encoder < Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder + MAXEncoder;
  }
  else if(Data->Encoder > Data->Last_Encoder)
  {
      res1 = Data->Encoder - Data->Last_Encoder - MAXEncoder;
  }
  res2 = Data->Encoder - Data->Last_Encoder;
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* transforms the Encoder data to tolangle */
	if(fabsf(res1) > fabsf(res2))
	{
		Data->Angle_sum += (float)res2/(MAXEncoder*Torque_Ratio)*360.f;
	}
	else
	{
		Data->Angle_sum += (float)res1/(MAXEncoder*Torque_Ratio)*360.f;
	}
  
  return Data->Angle_sum;
}
//------------------------------------------------------------------------------

/**
  * @brief  transform the Encoder(0-8192) to angle(-180-180)
  * @param  *Data        pointer to a Motor_Data_Typedef structure that 
	*					             contains the Data for the specified motor
  * @param  torque_ratio the specified motor torque ratio
  * @param  MAXEncoder   the specified motor max Encoder number
  * @retval angle
  */
float DJI_Motor_Encoder_To_Angle(DJI_Motor_Data_Typedef *Data,float torque_ratio,uint16_t MAXEncoder)
{	
  float Encoder_Err = 0.f;
  
  /* check the motor init */
  if(Data->Initlized != true)
  {
    /* update the last Encoder */
    Data->Last_Encoder = Data->Encoder;

    /* reset the angle */
    Data->Angle = Data->Encoder/(MAXEncoder*torque_ratio)*360.f;

    /* config the init flag */
    Data->Initlized = true;
  }
  
  Encoder_Err = Data->Encoder - Data->Last_Encoder;
  
  /* 0 -> MAXEncoder */		
  if(Encoder_Err > MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err - MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  /* MAXEncoder-> 0 */		
  else if(Encoder_Err < -MAXEncoder*0.5f)
  {
    Data->Angle += (float)(Encoder_Err + MAXEncoder)/(MAXEncoder*torque_ratio)*360.f;
  }
  else
  {
    Data->Angle += (float)(Encoder_Err)/(MAXEncoder*torque_ratio)*360.f;
  }
  
  /* update the last Encoder */
  Data->Last_Encoder = Data->Encoder;
  
  /* loop constrain */
  Data->Angle = F_Loop_Constrain(Data->Angle,-180.f,180.f);

  return Data->Angle;
}

static void MIT_motor_Position_To_Anglesum(MIT_motor_Data_Typedef *Data, float P_MAX)
{
    if(Data == NULL) return;

    if(Data->Initlized != true) return;

    float delta = Data->Position - Data->Last_Position;

    // gestione wrap-around 盤_MAX
    if(delta > P_MAX)
        delta -= 2.0f * P_MAX;
    else if(delta < -P_MAX)
        delta += 2.0f * P_MAX;

    Data->Angle_sum += delta;
    Data->Last_Position = Data->Position;
}

/**
  * @brief  Transmit enable disable save zero position Command to DM motor 
  * @param  *FDCAN_TxFrame：pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *MIT_motor：pointer to the MIT_motor
  * @param  CMD：Transmit Command  (DJI_Motor_Type_e)
  * @retval None
  */
void MIT_motor_Command(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,MIT_motor_Info_Typedef *MIT_motor,uint8_t CMD){

	 FDCAN_TxFrame->Header.Identifier = MIT_motor->FDCANFrame.TxIdentifier;
  	
	 FDCAN_TxFrame->Data[0] = 0xFF;
   FDCAN_TxFrame->Data[1] = 0xFF;
 	 FDCAN_TxFrame->Data[2] = 0xFF;
	 FDCAN_TxFrame->Data[3] = 0xFF;
	 FDCAN_TxFrame->Data[4] = 0xFF;
	 FDCAN_TxFrame->Data[5] = 0xFF;
	 FDCAN_TxFrame->Data[6] = 0xFF;
	
	 switch(CMD){
		 
		  case Motor_Enable :
	        FDCAN_TxFrame->Data[7] = 0xFC; 
	    break;
      
			case Motor_Disable :
	        FDCAN_TxFrame->Data[7] = 0xFD; 
      break;
      
			case Motor_Save_Zero_Position :
	        FDCAN_TxFrame->Data[7] = 0xFE; 
			break;
			
			default:
	    break;   
	}
	
   USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);

}

/**
  * @brief  CAN Transmit DM motor Information
  * @param  *FDCAN_TxFrame  pointer to the FDCAN_TxFrame_TypeDef.
  * @param  *MIT_motor  pointer to the MIT_motor
  * @param  Postion Velocity KP KD Torgue: Target
  * @retval None
  */
void MIT_motor_CAN_TxMessage(FDCAN_TxFrame_TypeDef *FDCAN_TxFrame,MIT_motor_Info_Typedef *MIT_motor,float Postion, float Velocity, float KP, float KD, float Torque){
	
   if(MIT_motor->Control_Mode == MIT){
		 
		 uint16_t Postion_Tmp,Velocity_Tmp,Torque_Tmp,KP_Tmp,KD_Tmp;
		 
		 Postion_Tmp  =  float_to_uint(Postion, -MIT_motor->Param_Range.P_MAX,MIT_motor->Param_Range.P_MAX,16) ;
		 Velocity_Tmp =  float_to_uint(Velocity,-MIT_motor->Param_Range.V_MAX,MIT_motor->Param_Range.V_MAX,12);
		 Torque_Tmp   =  float_to_uint(Torque,  -MIT_motor->Param_Range.T_MAX,MIT_motor->Param_Range.T_MAX,12);
		 KP_Tmp = float_to_uint(KP,0,500,12);
		 KD_Tmp = float_to_uint(KD,0,5,12);
		
		 FDCAN_TxFrame->Header.Identifier = MIT_motor->FDCANFrame.TxIdentifier;
		 
		 FDCAN_TxFrame->Data[0] = (uint8_t)(Postion_Tmp>>8);
		 FDCAN_TxFrame->Data[1] = (uint8_t)(Postion_Tmp);
		 FDCAN_TxFrame->Data[2] = (uint8_t)(Velocity_Tmp>>4);
		 FDCAN_TxFrame->Data[3] = (uint8_t)((Velocity_Tmp&0x0F)<<4) | (uint8_t)(KP_Tmp>>8);
		 FDCAN_TxFrame->Data[4] = (uint8_t)(KP_Tmp);
		 FDCAN_TxFrame->Data[5] = (uint8_t)(KD_Tmp>>4);
		 FDCAN_TxFrame->Data[6] = (uint8_t)((KD_Tmp&0x0F)<<4) | (uint8_t)(Torque_Tmp>>8);
		 FDCAN_TxFrame->Data[7] = (uint8_t)(Torque_Tmp);

	}else if(MIT_motor->Control_Mode == POSITION_VELOCITY){
	
		 uint8_t *Postion_Tmp,*Velocity_Tmp;
		
		 Postion_Tmp  = (uint8_t*) & Postion;
		 Velocity_Tmp = (uint8_t*) & Velocity;
		
	   FDCAN_TxFrame->Header.Identifier = MIT_motor->FDCANFrame.TxIdentifier + 0x100;
		
		 FDCAN_TxFrame->Data[0] = *(Postion_Tmp);
		 FDCAN_TxFrame->Data[1] = *(Postion_Tmp + 1);
		 FDCAN_TxFrame->Data[2] = *(Postion_Tmp + 2);
		 FDCAN_TxFrame->Data[3] = *(Postion_Tmp + 3);
	   FDCAN_TxFrame->Data[4] = *(Velocity_Tmp);
		 FDCAN_TxFrame->Data[5] = *(Velocity_Tmp + 1);
		 FDCAN_TxFrame->Data[6] = *(Velocity_Tmp + 2);
		 FDCAN_TxFrame->Data[7] = *(Velocity_Tmp + 3);
		
	}else if(MIT_motor->Control_Mode == VELOCITY){
	
	  uint8_t *Velocity_Tmp;
		Velocity_Tmp = (uint8_t*) & Velocity;
		
    FDCAN_TxFrame->Header.Identifier = MIT_motor->FDCANFrame.TxIdentifier + 0x200;
		
		FDCAN_TxFrame->Data[0] = *(Velocity_Tmp);
		FDCAN_TxFrame->Data[1] = *(Velocity_Tmp + 1);
		FDCAN_TxFrame->Data[2] = *(Velocity_Tmp + 2);
		FDCAN_TxFrame->Data[3] = *(Velocity_Tmp + 3);
		FDCAN_TxFrame->Data[4] = 0;
 		FDCAN_TxFrame->Data[5] = 0;
		FDCAN_TxFrame->Data[6] = 0;
		FDCAN_TxFrame->Data[7] = 0;

	}
	 
	  USER_FDCAN_AddMessageToTxFifoQ(FDCAN_TxFrame);

}
//------------------------------------------------------------------------------

/**
  * @brief  Update the MIT_motor Information
  * @param  Identifier:  pointer to the specifies the standard identifier.
  * @param  Rx_Buf:  pointer to the can receive data
  * @param  MIT_motor: pointer to a MIT_motor_Info_Typedef structure that contains the information of MIT_motor
  * @retval None
  */
void MIT_motor_Info_Update(uint32_t *Identifier, uint8_t *Rx_Buf, MIT_motor_Info_Typedef *MIT_motor)
{
    if(*Identifier != MIT_motor->FDCANFrame.RxIdentifier) return;

    MIT_motor->Data.State    = Rx_Buf[0]>>4;
    MIT_motor->Data.P_int    = ((uint16_t)(Rx_Buf[1])<<8) | ((uint16_t)(Rx_Buf[2]));
    MIT_motor->Data.V_int    = ((uint16_t)(Rx_Buf[3])<<4) | ((uint16_t)(Rx_Buf[4])>>4);
    MIT_motor->Data.T_int    = ((uint16_t)(Rx_Buf[4]&0xF)<<8) | ((uint16_t)(Rx_Buf[5]));

    MIT_motor->Data.Torque   = uint_to_float(MIT_motor->Data.T_int, -MIT_motor->Param_Range.T_MAX, MIT_motor->Param_Range.T_MAX, 12);
    MIT_motor->Data.Position = uint_to_float(MIT_motor->Data.P_int, -MIT_motor->Param_Range.P_MAX, MIT_motor->Param_Range.P_MAX, 16);
    MIT_motor->Data.Velocity = uint_to_float(MIT_motor->Data.V_int, -MIT_motor->Param_Range.V_MAX, MIT_motor->Param_Range.V_MAX, 12);

    MIT_motor->Data.Temperature_MOS   = (float)(Rx_Buf[6]);
    MIT_motor->Data.Temperature_Rotor = (float)(Rx_Buf[7]);

    MIT_motor_Position_To_Anglesum(&MIT_motor->Data, MIT_motor->Param_Range.P_MAX);
}
//------------------------------------------------------------------------------	
	
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
