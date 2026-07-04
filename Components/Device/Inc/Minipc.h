/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.h
  * @brief          : Jetson (CV) <-> MCU protocol over USB CDC (/dev/ttyACM0)
  * @date           : 2026/07/04
  * @version        : v3.0  ARC RMUL protocol (sp_vision auto_aim pipeline)
  ******************************************************************************
  * @attention
  * Raw packed little-endian structs, NO 'S'/'P' frame bytes and NO CRC16.
  * Must stay byte-identical to io/gimbal/gimbal.hpp on the Jetson:
  *   GimbalToVision  (MCU -> Jetson)  43 bytes
  *   VisionToGimbal  (Jetson -> MCU)  35 bytes
  * The Jetson re-synchronizes by field validation (quaternion norm + ranges);
  * the MCU does the same on its RX path.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef DEVICE_MINIPC_H
#define DEVICE_MINIPC_H

#include "stdint.h"

/* ============================================================
   WIRE FORMAT (must match Jetson io/gimbal/gimbal.hpp)
   ============================================================ */
#pragma pack(push, 1)

typedef struct
{
    uint8_t  mode;           /* 0: idle, 1: auto-aim (2/3 buff, unused)      */
    uint8_t  aim_color;      /* enemy color: 0 = blue enemy, 1 = red enemy   */
    float    q[4];           /* IMU quaternion, w x y z (normalized)         */
    float    yaw;            /* current gimbal yaw, rad (continuous)         */
    float    yaw_vel;        /* rad/s                                        */
    float    pitch;          /* current gimbal pitch, rad                    */
    float    pitch_vel;      /* rad/s                                        */
    float    bullet_speed;   /* m/s, 0 if unknown (Jetson uses fallback)     */
    uint16_t bullet_count;   /* cumulative shots fired (referee 0x0207)      */
    uint16_t self_HP;
    uint8_t  match_started;  /* 0/1 (game_progress == 4)                     */
} GimbalToVision_t;

typedef struct
{
    uint8_t mode;            /* 0: no control, 1: aim only, 2: aim + fire    */
    uint8_t is_self_color_red;

    float yaw;               /* rad, world/IMU frame, wrapped to [-pi, pi]   */
    float yaw_vel;           /* rad/s   (feedforward)                        */
    float yaw_acc;           /* rad/s^2 (feedforward)                        */
    float pitch;             /* rad, world/IMU frame                         */
    float pitch_vel;         /* rad/s                                        */
    float pitch_acc;         /* rad/s^2                                      */

    float forward_vel;       /* m/s, sentry only                             */
    float leftward_vel;      /* m/s, sentry only                             */
    uint8_t spintop_level;   /* 0 = no spin                                  */
} VisionToGimbal_t;

#pragma pack(pop)

#define NUM_BYTES_TX_MINIPC  43U
#define NUM_BYTES_RX_MINIPC  35U

/* Compile-time layout guards (array size is negative on mismatch) */
typedef uint8_t MiniPC_assert_tx_size[(sizeof(GimbalToVision_t) == NUM_BYTES_TX_MINIPC) ? 1 : -1];
typedef uint8_t MiniPC_assert_rx_size[(sizeof(VisionToGimbal_t) == NUM_BYTES_RX_MINIPC) ? 1 : -1];

/* ============================================================
   RX STATE
   ============================================================ */

/* Last valid command, stored in a naturally-aligned struct (safe field reads) */
typedef struct
{
    uint8_t mode;
    uint8_t is_self_color_red;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    float forward_vel;
    float leftward_vel;
    uint8_t spintop_level;
} cv_command_t;

extern cv_command_t MiniPC_CV_Cmd;              /* last valid packet (updated in USB IRQ) */
extern volatile uint32_t MiniPC_Rx_Valid_Count; /* valid packets accepted   */
extern volatile uint32_t MiniPC_Rx_Resync_Count;/* bytes dropped on resync  */

/* Freshness windows */
#define MINIPC_CV_TIMEOUT_MS       400U  /* general data hold (idle keepalive is 200 ms) */
#define MINIPC_CV_FIRE_TIMEOUT_MS  150U  /* fire flag only (aiming stream is ~250 Hz)    */

/* ============================================================
   NAMED ACCESS (kept for state_machine.c / chassis_control.c)
   ============================================================ */
#define yaw_cv          (MiniPC_CV_Cmd.yaw)
#define pitch_cv        (MiniPC_CV_Cmd.pitch)
#define shoot_flag_cv   (MiniPC_CV_Fire())
#define fwd_bwd_cv      (MiniPC_CV_Fresh() ? MiniPC_CV_Cmd.forward_vel  : 0.0f)
#define left_right_cv   (MiniPC_CV_Fresh() ? MiniPC_CV_Cmd.leftward_vel : 0.0f)
#define flag_rot        ((MiniPC_CV_Fresh() && MiniPC_CV_Cmd.spintop_level > 0) ? 1 : 0)

/* ============================================================
   FUNCTIONS
   ============================================================ */
void MiniPC_Transmit_Info(void);
void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len);

uint8_t MiniPC_CV_Fresh(void);                  /* valid packet within MINIPC_CV_TIMEOUT_MS      */
uint8_t MiniPC_CV_Fire(void);                   /* mode == 2 and within MINIPC_CV_FIRE_TIMEOUT_MS */
uint8_t MiniPC_Get_CV_Cmd(cv_command_t *out);   /* IRQ-safe snapshot; 1 if fresh and mode >= 1    */

#endif /* DEVICE_MINIPC_H */
