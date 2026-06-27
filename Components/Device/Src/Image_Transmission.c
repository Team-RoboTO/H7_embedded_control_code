/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Image_Transmission.c
  * @brief          : Image_Transmission_Info interfaces functions 
  * @version        : v1.0
  ******************************************************************************
  * @attention      : to be tested
  ******************************************************************************
  */
/* USER CODE END Header */

#include "Image_Transmission.h"
#include "remote_control.h"
#include "CRC.h"

__attribute__((section (".AXI_SRAM"))) uint8_t Image_Trans_MultiRx_Buff[2][39];

Image_Transmission_Info_TypeDef Image_Transmission_Info;
VT13_Info_TypeDef VT13_Info;

static int16_t bit8TObit16(uint8_t change_info[2]);
static int16_t last_button_state[3] = {0};

void Image_Transmission_Info_Update(uint8_t *Buff, uint16_t Size){

	uint16_t i = 0;
	while (i < Size - 5) // Minimum header + some data
	{
		if(Buff[i] == 0xA5)
		{
			if(Verify_CRC8_Check_Sum(&Buff[i], 5) == true)
			{
				uint16_t data_len = (uint16_t)(Buff[i+2]<<8 | Buff[i+1]);
				uint16_t frame_len = data_len + FrameHeader_Length + CMDID_Length + CRC16_Length;
				
				if(i + frame_len <= Size && Verify_CRC16_Check_Sum(&Buff[i], frame_len) == true)
				{
					uint16_t cmd_id = (uint16_t)(Buff[i+6]<<8 | Buff[i+5]);
					
					#ifdef REMOTE_CONTROL_ID
					if(cmd_id == REMOTE_CONTROL_ID && data_len == 12) // 0x0304 is usually 12 bytes of data
					{
						Image_Transmission_Info.remote_control.mouse_x = bit8TObit16(&Buff[i + FrameHeader_Length + CMDID_Length]);
						Image_Transmission_Info.remote_control.mouse_y = bit8TObit16(&Buff[i + FrameHeader_Length + CMDID_Length + 2]);
						Image_Transmission_Info.remote_control.mouse_z = bit8TObit16(&Buff[i + FrameHeader_Length + CMDID_Length + 4]);
						Image_Transmission_Info.remote_control.left_button_down  = Buff[i + FrameHeader_Length + CMDID_Length + 6];
						Image_Transmission_Info.remote_control.right_button_down = Buff[i + FrameHeader_Length + CMDID_Length + 7];
						Image_Transmission_Info.remote_control.Key.keyboard_value = bit8TObit16(&Buff[i + FrameHeader_Length + CMDID_Length + 8]);
						
						// Synchronize to RC_info for control tasks
						RC_info.Mouse.X = Image_Transmission_Info.remote_control.mouse_x;
						RC_info.Mouse.Y = Image_Transmission_Info.remote_control.mouse_y;
						RC_info.Mouse.Z = Image_Transmission_Info.remote_control.mouse_z;
						RC_info.Mouse.Press_L = Image_Transmission_Info.remote_control.left_button_down;
						RC_info.Mouse.Press_R = Image_Transmission_Info.remote_control.right_button_down;
						RC_info.Key.V = Image_Transmission_Info.remote_control.Key.keyboard_value;
					}
					#endif
					
					i += frame_len;
					continue;
				}
			}
		}
		else if(Buff[i] == 0xA9)
		{
			if(i + 21 <= Size && Buff[i+1] == 0x53)
			{
				if(Verify_CRC16_Check_Sum(&Buff[i], 21) == true)
				{
					VT13_Info_Update(&Buff[i], &VT13_Info);
					i += 21;
					continue;
				}
			}
		}
		
		i++;
	}
}

void VT13_Info_Update(uint8_t *Buff ,VT13_Info_TypeDef *VT13_Info){
	
	   if(Verify_CRC16_Check_Sum(&Buff[0],21) == true){
		 
		    VT13_Info->RC.Channel[0] = ( (Buff[2]) | (Buff[3] << 8) )& 0x07FF;                           
        VT13_Info->RC.Channel[1] = ( (Buff[3] >> 3) | (Buff[4] << 5 ) ) & 0x07FF;                            
        VT13_Info->RC.Channel[2] = ( (Buff[4] >> 6) | (Buff[5] << 2 ) | (Buff[6] << 10) ) & 0x07FF;      
        VT13_Info->RC.Channel[3] = ( (Buff[6] >> 1) | (Buff[7] << 7 ) ) & 0x07FF;  


			  VT13_Info->RC.Switch     = (  Buff[7] >> 4 ) & 0x03;
			 
				// We want these three to be on/off buttons

				if( (  Buff[7] >> 6 ) & 0x01  && last_button_state[0] == 0) {
					VT13_Info->RC.Stop  = !VT13_Info->RC.Stop;
				}
				last_button_state[0] = (  Buff[7] >> 6 ) & 0x01 ;
				
				VT13_Info->RC.Left = (  Buff[7] >> 7 ) & 0x01;
				
				if( (  Buff[8]  ) & 0x01 && last_button_state[2] == 0) {
					 VT13_Info->RC.Right = ! VT13_Info->RC.Right ;
				}
				last_button_state[2] = (  Buff[8]  ) & 0x01;
			 
			
			  VT13_Info->RC.Wheel      = ( (Buff[8] >> 1) | (Buff[9] << 7 ) ) & 0x07FF; 
			  VT13_Info->RC.Trigger    = (  Buff[9] >> 4 ) & 0x01;
			 
			  VT13_Info->RC.Channel[0] -= 1024;
		    VT13_Info->RC.Channel[1] -= 1024;
        VT13_Info->RC.Channel[2] -= 1024;
		    VT13_Info->RC.Channel[3] -= 1024;
		    VT13_Info->RC.Wheel      -= 1024;
			 
			  VT13_Info->Mouse.X = (Buff[10]  | (Buff[11] << 8));
			  VT13_Info->Mouse.Y = (Buff[12]  | (Buff[13] << 8));
			  VT13_Info->Mouse.Z = (Buff[14]  | (Buff[15] << 8));
			 
			  VT13_Info->Mouse.Press_L = (Buff[16]) & 0x03;
				VT13_Info->Mouse.Press_R = (Buff[16] >> 2) & 0x03;
				VT13_Info->Mouse.Press_M = (Buff[16] >> 4) & 0x03;

        VT13_Info->Key.V = (Buff[17] | (Buff[18] << 8));
				
				#if IS_VT13_REMOTE
	
						RC_info = *VT13_Info;                          // direct struct copy, same layout
						
				#endif
		 }	 
}


void Robot_Data_to_Custom_(uint8_t *Data){

    for(uint8_t i = 0; i<30; i++){
			
		 Image_Transmission_Info.robot_custom_data.data[i] =  Data[i];
		
		}
    
		HAL_UART_Transmit_DMA(&huart7,Image_Transmission_Info.robot_custom_data.data,30);
}

/**
 * @brief  transform the bit8 to bit16
*/
static int16_t bit8TObit16(uint8_t change_info[2])
{
	union
	{
    int16_t  bit16;
		uint8_t  byte[2];
	}u16val;

  u16val.byte[0] = change_info[0];
  u16val.byte[1] = change_info[1];

	return u16val.bit16;
}