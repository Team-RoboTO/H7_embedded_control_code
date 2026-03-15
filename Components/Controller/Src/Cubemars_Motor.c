#include "cubemars_motor.h"
#include <string.h>
#include "stdint.h"

/**
 * @file    cubemars.c
 * @brief   CubeMars AK40-10 � CAN protocol + motor control
 */

  /****************************/
 /*   INTERNAL HELPERS       */
/****************************/

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


// CubeMars motor control - buffer formation
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
 buffer[(*index)++] = number >> 8;
 buffer[(*index)++] = number;
 }

// CubeMars motor control - CAN transmit MODIFIED (different from the one proposed from CubeMars)
void comm_can_transmit_eid_MODIFIED(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    FDCAN1_TxFrame.Header.Identifier    = id;
    FDCAN1_TxFrame.Header.IdType        = FDCAN_EXTENDED_ID;
    FDCAN1_TxFrame.Header.TxFrameType   = FDCAN_DATA_FRAME;
    FDCAN1_TxFrame.Header.DataLength    = (len <= 4) ? FDCAN_DLC_BYTES_4 : FDCAN_DLC_BYTES_8;
    FDCAN1_TxFrame.Header.FDFormat      = FDCAN_CLASSIC_CAN;
    FDCAN1_TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
    for (i = 0; i < len; i++) FDCAN1_TxFrame.Data[i] = data[i];
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
}

// CubeMars current control
void CAN_Tx_gimbal_cubemars( 
    float current_ampere_pitch) {
	  
  	int32_t yaw_send_index = 0;
		uint8_t yaw_buffer[4];
			
		int32_t pitch_send_index = 0;
		uint8_t pitch_buffer[4];
		buffer_append_int32(pitch_buffer, (int32_t)(current_ampere_pitch), &pitch_send_index);
		comm_can_transmit_eid_MODIFIED(GIMBAL_PITCH_CUBEMARS_ID_CAN |
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), pitch_buffer, pitch_send_index);
			
}	
		

// CubeMars motor control - Set pos command MODIFIED
// The position value is of type int32, and the range is -360000000-360000000, representing -36000?-36000?
// [FOR AK60 AND AK70]Input position range [0, inf] (but max position displayed from data receive is 3200). If pos = 3.6 --> 1 motor rotation, If pos = 36 --> 10 motor rotation
void CAN_Tx_gimbal_cm_position(float position) {
	
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(position * 10000.0), &send_index);
	comm_can_transmit_eid_MODIFIED(GIMBAL_PITCH_CUBEMARS_ID_CAN |
			((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
	
}

// CubeMars motor control - Velocity (rpm) command MODIFIED
// rpm range -100000, +100000
void CAN_Tx_gimbal_cm_rpm(float rpm){ 
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	comm_can_transmit_eid_MODIFIED(GIMBAL_PITCH_CUBEMARS_ID_CAN |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}


// CubeMars motor control - Position with maximum speed and maximum acceleration command MODIFIED
// position: int32, range-360000000~360000000 representing-36000?~36000?
// max speed: int16, range-32768~32767 representing-327680~-327680 electrical RPM
// max acceleration: int16, range 0~32767, representing 0~327670, 1 unit equals 10 electrical RPM/s
void CAN_Tx_gimbal_cm_pos_spd_acc(float position, int16_t max_speed, int16_t acceleration) {
	int32_t send_index = 0;
	uint8_t buffer[8];

	// Append position (4 bytes)
	buffer_append_int32(buffer, (int32_t)(position * 10000.0), &send_index);

	// Append speed (2 bytes)
	buffer_append_int16(buffer, (int16_t)(max_speed / 10.0), (int16_t*)&send_index);

	// Append acceleration (2 bytes)
	buffer_append_int16(buffer, (int16_t)(acceleration / 10.0), (int16_t*)&send_index);

	comm_can_transmit_eid_MODIFIED(GIMBAL_PITCH_CUBEMARS_ID_CAN | 
			((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}

// CubeMars set zero position of motor (yaw)
void CAN_Tx_yaw_cubemars_set_origin() {
	
		int32_t send_index = 0;
		uint8_t buffer[4];
		buffer_append_int32(buffer, (int32_t)(1), &send_index);
		comm_can_transmit_eid_MODIFIED(GIMBAL_YAW_CUBEMARS_ID_CAN |
			((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}	

// CubeMars set zero position of motor (pitch)
void CAN_Tx_pitch_cubemars_set_origin() {
	
		int32_t send_index = 0;
		uint8_t buffer[4];
		buffer_append_int32(buffer, (int32_t)(1), &send_index);
		comm_can_transmit_eid_MODIFIED(GIMBAL_PITCH_CUBEMARS_ID_CAN |
			((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), buffer, send_index);
}

// **************************************** function definitions - MIT MODE FOR CUBEMARS ***************************************

// MIT Mode transmit function
void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len)
{
    uint8_t i;
    FDCAN1_TxFrame.Header.Identifier    = id;
    FDCAN1_TxFrame.Header.IdType        = FDCAN_STANDARD_ID;
    FDCAN1_TxFrame.Header.TxFrameType   = FDCAN_DATA_FRAME;
    FDCAN1_TxFrame.Header.DataLength    = FDCAN_DLC_BYTES_8;
    FDCAN1_TxFrame.Header.FDFormat      = FDCAN_CLASSIC_CAN;
    FDCAN1_TxFrame.Header.BitRateSwitch = FDCAN_BRS_OFF;
    for (i = 0; i < len; i++) FDCAN1_TxFrame.Data[i] = data[i];
    USER_FDCAN_AddMessageToTxFifoQ(&FDCAN1_TxFrame);
}

// Enter MIT Motor Control Mode
void CAN_Tx_MIT_Enter_Control_Mode(void) {
    uint8_t buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    comm_can_transmit_sid(GIMBAL_PITCH_CUBEMARS_ID_CAN, buffer, 8);
}

// Exit MIT Motor Control Mode
void CAN_Tx_MIT_Exit_Control_Mode(void) {
    uint8_t buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    comm_can_transmit_sid(GIMBAL_PITCH_CUBEMARS_ID_CAN, buffer, 8);
}

// Set current position as zero in MIT mode
void CAN_Tx_MIT_Set_Zero_Position(void) {
    uint8_t buffer[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    comm_can_transmit_sid(GIMBAL_PITCH_CUBEMARS_ID_CAN, buffer, 8);
}

// MIT Mode Control Command
void CAN_Tx_MIT_Control(float p_des, float v_des, float kp, float kd, float t_ff) {
    uint8_t buffer[8];
    
    // Limit data to be within bounds
    p_des = fminf(fmaxf(CM_P_MIN, p_des), CM_P_MAX);
    v_des = fminf(fmaxf(CM_V_MIN, v_des), CM_V_MAX);
    kp = fminf(fmaxf(CM_KP_MIN, kp), CM_KP_MAX);
    kd = fminf(fmaxf(CM_KD_MIN, kd), CM_KD_MAX);
    t_ff = fminf(fmaxf(CM_T_MIN, t_ff), CM_T_MAX);
    
    // Convert floats to unsigned ints
    uint16_t p_int = float_to_uint(p_des, CM_P_MIN, CM_P_MAX, 16);
    uint16_t v_int = float_to_uint(v_des, CM_V_MIN, CM_V_MAX, 12);
    uint16_t kp_int = float_to_uint(kp, CM_KP_MIN, CM_KP_MAX, 12);
    uint16_t kd_int = float_to_uint(kd, CM_KD_MIN, CM_KD_MAX, 12);
    uint16_t t_int = float_to_uint(t_ff, CM_T_MIN, CM_T_MAX, 12);
    
    // Pack ints into the buffer
    buffer[0] = (uint8_t)(p_int >> 8);                          // Position High 8
    buffer[1] = (uint8_t)(p_int & 0xFF);                        // Position Low 8
    buffer[2] = (uint8_t)(v_int >> 4);                          // Speed High 8 bits
    buffer[3] = (uint8_t)(((v_int & 0xF) << 4) | (kp_int >> 8)); // Speed Low 4 | KP High 4
    buffer[4] = (uint8_t)(kp_int & 0xFF);                       // KP Low 8 bits
    buffer[5] = (uint8_t)(kd_int >> 4);                         // Kd High 8 bits
    buffer[6] = (uint8_t)(((kd_int & 0xF) << 4) | (t_int >> 8)); // KD Low 4 | Torque High 4
    buffer[7] = (uint8_t)(t_int & 0xFF);                        // Torque Low 8 bits
    
    comm_can_transmit_sid(GIMBAL_PITCH_CUBEMARS_ID_CAN, buffer, 8);
}

// Parse MIT mode motor feedback
void CAN_Rx_MIT_Parse(uint8_t *data, float *position, float *velocity, float *torque, uint8_t *temperature) {
    // Unpack ints from CAN buffer
    uint16_t p_int = ((uint16_t)data[1] << 8) | data[2];
    uint16_t v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    uint16_t i_int = (((uint16_t)data[4] & 0xF) << 8) | data[5];
    uint8_t T_int = data[6];
    
    // Convert ints to floats
    *position = uint_to_float(p_int, CM_P_MIN, CM_P_MAX, 16);
    *velocity = uint_to_float(v_int, CM_V_MIN, CM_V_MAX, 12);
    *torque = uint_to_float(i_int, CM_T_MIN, CM_T_MAX, 12);
    *temperature = T_int;  // Temperature stored directly as uint8_t (after -40 offset applied in insert function)
}
