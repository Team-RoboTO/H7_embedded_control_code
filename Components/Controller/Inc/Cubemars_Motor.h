#ifndef CUBEMARS_MOTOR_H
#define CUBEMARS_MOTOR_H

#include "main.h"
#include "PID.h"
#include "config.h"
#include "stm32h723xx.h"
#include "bsp_can.h"
#include "motor.h"

/**
 * @file    cubemars.h
 * @brief   CubeMars AK40-10 � CAN protocol + motor control
 *          STM32H7 FDCAN, MIT mini-cheetah protocol, classic CAN, 1Mbps
 *
 */

// **************************** Parameter Definitions - MIT MODE FOR CUBEMARS *******************
// MIT Mode Parameters for all Cubemars motors
#define CM_P_MIN -12.5f
#define CM_P_MAX  12.5f
#define CM_V_MIN -50.0f
#define CM_V_MAX  50.0f
#define CM_T_MIN -25.0f
#define CM_T_MAX  25.0f
#define CM_KP_MIN 0.0f
#define CM_KP_MAX 500.0f
#define CM_KD_MIN 0.0f
#define CM_KD_MAX 5.0f

#define AK40_POS_MIN    CM_P_MIN
#define AK40_POS_MAX    CM_P_MAX
#define AK40_VEL_MIN    CM_V_MIN
#define AK40_VEL_MAX    CM_V_MAX
#define AK40_TORQUE_MIN CM_T_MIN
#define AK40_TORQUE_MAX CM_T_MAX

// **************************** End - MIT MODE FOR CUBEMARS *******************

///* MIT bit field widths */
//#define MIT_POS_BITS        16
//#define MIT_VEL_BITS        12
//#define MIT_KP_BITS         12
//#define MIT_KD_BITS         12
//#define MIT_TORQUE_BITS     12
//#define CUBEMARS_CAN_FRAME_BYTES    8U

/* Special command words */
#define CUBEMARS_CMD_ENTER_CONTROL  0xFFFFFFFFFFFFFFFFULL
#define CUBEMARS_CMD_EXIT_CONTROL   0xFFFFFFFFFFFFFFFEULL
#define CUBEMARS_CMD_SET_ZERO       0xFFFFFFFFFFFFFFF5ULL

  /************************/
 /*   CAN TX FUNCTIONS   */
/************************/

void CAN_Tx_chassis_wheels(
    int16_t current_ampere_motor_1,
    int16_t current_ampere_motor_2,
    int16_t current_ampere_motor_3,
    int16_t current_ampere_motor_4);

void CAN_Tx_gimbal(
    int16_t current_ampere_yaw,
    int16_t current_ampere_pitch);

void CAN_Tx_shoot_wheels_rev(
    int16_t current_ampere_shoot_wheel_left,
    int16_t current_ampere_shoot_wheel_right,
    int16_t current_ampere_rev);


void CAN_Tx_gimbal_cubemars(
    float current_ampere_pitch);

void CAN_Tx_gimbal_cm_position(
		float position);

void CAN_Tx_gimbal_cm_pos_spd_acc(
		float position, 
		int16_t max_speed, 
		int16_t acceleration);

void CAN_Tx_yaw_cubemars_set_origin();

void CAN_Tx_pitch_cubemars_set_origin();

void CAN_Tx_shoot_wheels_rev_double_barrel(
    int16_t current_ampere_shoot_wheel_left,
    int16_t current_ampere_shoot_wheel_right,
    int16_t current_ampere_rev);

void CAN_Tx_shoot_wheels_rev(
    int16_t current_ampere_shoot_wheel_left,
    int16_t current_ampere_shoot_wheel_right,
    int16_t current_ampere_rev);

// CubeMars CAN commands modes
typedef enum {
	CAN_PACKET_SET_DUTY = 0,  //????g?
	CAN_PACKET_SET_CURRENT,   //??????g?
	CAN_PACKET_SET_CURRENT_BRAKE,  //???g?
	CAN_PACKET_SET_RPM,            //???g?
	CAN_PACKET_SET_POS,            //???g?
	CAN_PACKET_SET_ORIGIN_HERE,    //???????
	CAN_PACKET_SET_POS_SPD         //???????g?
} CAN_PACKET_ID;

// CubeMars Functions declaration
void CM_motor_receive(float* motor_pos,float* motor_spd,float* cur,int8_t* temp,int8_t* error, uint8_t rx_message[]);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index);
void comm_can_transmit_eid_MODIFIED(uint32_t id, const uint8_t *data, uint8_t len);
void CAN_Tx_MIT_Enter_Control_Mode(void);
void CAN_Tx_MIT_Exit_Control_Mode(void);
void CAN_Tx_MIT_Set_Zero_Position(void);
void CAN_Tx_MIT_Control(float p_des, float v_des, float kp, float kd, float t_ff);


#endif /* CUBEMARS_MOTOR_H */