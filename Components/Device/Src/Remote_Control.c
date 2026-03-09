/* Includes ------------------------------------------------------------------*/
#include "Remote_Control.h"
#include "ramp.h"

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
 Remote_Info_Typedef remote_ctrl={
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
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channels — data starts at byte 2 (after 0xA9 0x53 header) */
    remote_ctrl->rc.right_h = (( sbus_buf[2]        | (sbus_buf[3] << 8))                          & 0x07FF) - RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.right_v = (((sbus_buf[3] >> 3)  | (sbus_buf[4] << 5))                          & 0x07FF) - RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.left_v  = (((sbus_buf[4] >> 6)  | (sbus_buf[5] << 2)  | (sbus_buf[6] << 10))  & 0x07FF) - RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.left_h  = (((sbus_buf[6] >> 1)  | (sbus_buf[7] << 7))                          & 0x07FF) - RC_CH_VALUE_OFFSET;

    /* Buttons — packed across bytes 7-9 */
    remote_ctrl->rc.mode_switch =  (sbus_buf[7] >> 4) & 0x03;
    remote_ctrl->rc.pause       =  (sbus_buf[7] >> 6) & 0x01;
    remote_ctrl->rc.custom_l    =  (sbus_buf[7] >> 7) & 0x01;
    remote_ctrl->rc.custom_r    =   sbus_buf[8]        & 0x01;
    remote_ctrl->rc.dial        = (((sbus_buf[8] >> 1) | (sbus_buf[9] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.trigger     =  (sbus_buf[9] >> 4) & 0x01;

    /* Mouse — bytes 10-16 */
    remote_ctrl->mouse.x       = sbus_buf[10] | (sbus_buf[11] << 8);
    remote_ctrl->mouse.y       = sbus_buf[12] | (sbus_buf[13] << 8);
    remote_ctrl->mouse.z       = sbus_buf[14] | (sbus_buf[15] << 8);
    remote_ctrl->mouse.press_l =  sbus_buf[16]        & 0x01;
    remote_ctrl->mouse.press_r = (sbus_buf[16] >> 2)  & 0x01;

    /* Keyboard — bytes 17-18 */
    remote_ctrl->key.v = sbus_buf[17] | (sbus_buf[18] << 8);

    remote_ctrl->online_cnt = 0xFAU;
    remote_ctrl->rc_lost = false;
}
//------------------------------------------------------------------------------

/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl)
{
    if(remote_ctrl->online_cnt <= 0x32U)
    {
        memset(remote_ctrl, 0, sizeof(Remote_Info_Typedef));

        /* reset sticks and dial to center so they read 0 after the -1024 offset */
        remote_ctrl->rc.right_h = 1024U;
        remote_ctrl->rc.right_v = 1024U;
        remote_ctrl->rc.left_v  = 1024U;
        remote_ctrl->rc.left_h  = 1024U;
        remote_ctrl->rc.dial    = 1024U;

        remote_ctrl->rc_lost = true;
    }
    else if(remote_ctrl->online_cnt > 0)
    {
        remote_ctrl->online_cnt--;
    }
}
//------------------------------------------------------------------------------