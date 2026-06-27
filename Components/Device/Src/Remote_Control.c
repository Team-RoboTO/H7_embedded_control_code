/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
 
 VT13_Info_TypeDef RC_info = {0};
 
 NDJ6_Info_Typedef NDJ6_info={
	.online_cnt = 0xFAU,
	.rc_lost = true,
};

/**
 * @brief remote control usart RxDMA MultiBuffer (cache-line aligned, padded to 32 bytes)
 */
__attribute__((section (".AXI_SRAM"), aligned(32))) uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_ALIGNED];
 
/* Private variables ---------------------------------------------------------*/
/**
 * @brief structure that contains the information of keyboard
 */
KeyBoard_Info_Typedef KeyBoard_Info;

/* Private function prototypes -----------------------------------------------*/
/**
  * @brief  Update the status of keyboard
  */
static void Key_Status_Update(KeyBoard_Info_Typedef *KeyInfo,bool KeyBoard_Status);

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, NDJ6_Info_Typedef *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;
	    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
    
		/* reset the online count */
		remote_ctrl->online_cnt = 0xFAU;
		
		/* reset the lost flag */
		remote_ctrl->rc_lost = false;
		
		#if IS_NDJ6_REMOTE
		RC_info.RC.Channel[0] = NDJ6_info.rc.ch[0];
    RC_info.RC.Channel[1] = NDJ6_info.rc.ch[1];
    RC_info.RC.Channel[2] = NDJ6_info.rc.ch[2];
    RC_info.RC.Channel[3] = NDJ6_info.rc.ch[3];
    RC_info.RC.Wheel       = NDJ6_info.rc.ch[4];

    RC_info.RC.Switch      = NDJ6_info.rc.s[0];
    //RC_info.RC.Right       = NDJ6_info.rc.s[1];

    RC_info.Mouse.X        = NDJ6_info.mouse.x;
    RC_info.Mouse.Y        = NDJ6_info.mouse.y;
    RC_info.Mouse.Z        = NDJ6_info.mouse.z;
    RC_info.Mouse.Press_L  = NDJ6_info.mouse.press_l;
    RC_info.Mouse.Press_R  = NDJ6_info.mouse.press_r;

    RC_info.Key.V          = NDJ6_info.key.v;
		#endif
}
//------------------------------------------------------------------------------

/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(NDJ6_Info_Typedef *remote_ctrl)
{
    if(remote_ctrl->online_cnt <= 0x32U)
    {
        memset(remote_ctrl, 0, sizeof(NDJ6_Info_Typedef));

        /* reset sticks and dial to center so they read 0 after the -1024 offset */
        remote_ctrl->rc.ch[0] = 1024U;
        remote_ctrl->rc.ch[0] = 1024U;
        remote_ctrl->rc.ch[0] = 1024U;
        remote_ctrl->rc.ch[0] = 1024U;
        remote_ctrl->rc.ch[0] = 1024U;

        remote_ctrl->rc_lost = true;
    }
    else if(remote_ctrl->online_cnt > 0)
    {
        remote_ctrl->online_cnt--;
    }
}

//------------------------------------------------------------------------------