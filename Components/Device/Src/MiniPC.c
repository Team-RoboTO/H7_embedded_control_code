/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : MiniPC.c
  * @brief          : Jetson (CV) <-> MCU protocol over USB CDC
  * @date           : 2026/07/04
  * @version        : v3.0  ARC RMUL protocol (sp_vision auto_aim pipeline)
  ******************************************************************************
  * @attention
  * TX: GimbalToVision (43 B) sent from USB_MiniPC_Task at 1 kHz.
  *     Every field is range-guarded so the packet always passes the Jetson's
  *     valid_packet() check (which replaces header + CRC for resync).
  * RX: VisionToGimbal (35 B) parsed in USB IRQ context (CDC_Receive_HS).
  *     A byte-stream accumulator with field validation re-aligns the stream
  *     if a transfer boundary is ever lost.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "MiniPC.h"
#include "usbd_cdc_if.h"
#include "Referee_System.h"
#include "INS_task.h"
#include "Quaternion.h"
#include "state_machine.h"
#include "Robot_config.h"
#include "control_utils.h"
#include <math.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

/* Ping-pong TX buffers: never rewrite a buffer that may still be in DMA */
static uint8_t Tx_buf[2][NUM_BYTES_TX_MINIPC];
static uint8_t Tx_buf_idx = 0;

/* RX stream accumulator */
static uint8_t  Rx_stream[2 * NUM_BYTES_RX_MINIPC];
static uint16_t Rx_stream_len = 0;

cv_command_t MiniPC_CV_Cmd = {0};
volatile uint32_t MiniPC_Rx_Valid_Count  = 0;
volatile uint32_t MiniPC_Rx_Resync_Count = 0;
static volatile uint32_t Rx_last_tick = 0;

/* ============================================================
   HELPERS
   ============================================================ */

/* true if v is a real number within [-lim, +lim] (NaN/inf rejected) */
static uint8_t f32_ok(float v, float lim)
{
    return (v == v) && (v >= -lim) && (v <= lim);
}

/* ============================================================
   TX: MCU -> Jetson
   ============================================================ */

static void MiniPC_Pack_Tx(GimbalToVision_t *p)
{
    /* mode: 1 only when the operator/state machine has auto-aim engaged */
    p->mode = (state_remote_commands != COMMANDS_STOP &&
               state_gimbal == GIMBAL_AUTO_AIM) ? 1 : 0;

    /* enemy color from referee robot_id (1..11 red self, 101..111 blue self).
       Referee offline (id 0) defaults to blue enemy, i.e. red self. */
    p->aim_color = (Referee_System_Info.robot_status.robot_id > 100) ? 1 : 0;

    /* IMU quaternion (w x y z), re-normalized so the Jetson norm check
       (|q|^2 within 1 +- 0.05) always passes */
    float qw = Quaternion_Info.quat[0];
    float qx = Quaternion_Info.quat[1];
    float qy = Quaternion_Info.quat[2];
    float qz = Quaternion_Info.quat[3];
    float norm2 = qw*qw + qx*qx + qy*qy + qz*qz;
    if (norm2 > 1e-6f && norm2 == norm2) {
        float inv = 1.0f / sqrtf(norm2);
        p->q[0] = qw * inv;
        p->q[1] = qx * inv;
        p->q[2] = qy * inv;
        p->q[3] = qz * inv;
    } else {
        p->q[0] = 1.0f;
        p->q[1] = p->q[2] = p->q[3] = 0.0f;
    }

    /* gimbal state: same sources as control_loop_gimbal() */
    p->yaw     = INS_Info.Yaw_TolAngle * DEG_TO_RAD;      /* continuous, rad */
    p->yaw_vel = INS_Info.Gyro[IMU_GYRO_INDEX_YAW];       /* rad/s           */
#if IS_STD || IS_SENTRY
    p->pitch     = INS_Info.Roll_Angle * DEG_TO_RAD;      /* IMU rotated 90deg */
    p->pitch_vel = INS_Info.Gyro[IMU_GYRO_INDEX_ROLL];
#elif IS_HERO
    p->pitch     = INS_Info.Pitch_Angle * DEG_TO_RAD;
    p->pitch_vel = INS_Info.Gyro[IMU_GYRO_INDEX_PITCH];
#endif
    if (!f32_ok(p->yaw, 1e6f))       p->yaw = 0.0f;
    if (!f32_ok(p->yaw_vel, 1e3f))   p->yaw_vel = 0.0f;
    if (!f32_ok(p->pitch, 3.14f))    p->pitch = 0.0f;
    if (!f32_ok(p->pitch_vel, 1e3f)) p->pitch_vel = 0.0f;

    /* bullet speed from referee 0x0207; 0 makes the Jetson use its
       configured fallback_bullet_speed. Must stay in [0, 50). */
    float v0 = Referee_System_Info.shoot_data.projectile_speed;
    if (!(v0 == v0) || v0 < 0.0f || v0 >= 50.0f) v0 = 0.0f;
    p->bullet_speed = v0;

    p->bullet_count  = Referee_System_Info.shoot_data.shot_count;
    p->self_HP       = Referee_System_Info.robot_status.current_HP;
    p->match_started = (Referee_System_Info.game_status.game_progress == 4) ? 1 : 0;
}

/**
  * @brief  Pack gimbal/referee state and send one 43-byte packet over USB CDC.
  * @note   Called from USB_MiniPC_Task at 1 kHz. Skips the cycle (keeps the
  *         freshly filled buffer for the next attempt) if USB is busy.
  */
void MiniPC_Transmit_Info(void)
{
    GimbalToVision_t p;
    MiniPC_Pack_Tx(&p);

    memcpy(Tx_buf[Tx_buf_idx], &p, NUM_BYTES_TX_MINIPC);

    if (CDC_Transmit_HS(Tx_buf[Tx_buf_idx], NUM_BYTES_TX_MINIPC) == USBD_OK) {
        Tx_buf_idx ^= 1;
    }
}

/* ============================================================
   RX: Jetson -> MCU
   ============================================================ */

/* Field validation stands in for the removed 'S''P' header + CRC16:
   a misaligned 35-byte window essentially never passes all checks. */
static uint8_t MiniPC_Rx_Valid(const VisionToGimbal_t *p)
{
    if (p->mode > 2) return 0;
    if (p->is_self_color_red > 1) return 0;
    if (p->spintop_level > 10) return 0;

    if (!f32_ok(p->yaw, 10.0f))        return 0;  /* Jetson sends [-pi, pi]  */
    if (!f32_ok(p->pitch, 3.15f))      return 0;
    if (!f32_ok(p->yaw_vel, 50.0f))    return 0;
    if (!f32_ok(p->pitch_vel, 50.0f))  return 0;
    if (!f32_ok(p->yaw_acc, 1e4f))     return 0;
    if (!f32_ok(p->pitch_acc, 1e4f))   return 0;
    if (!f32_ok(p->forward_vel, 20.0f))  return 0;
    if (!f32_ok(p->leftward_vel, 20.0f)) return 0;

    return 1;
}

static void MiniPC_Rx_Accept(const VisionToGimbal_t *p)
{
    /* Runs in USB IRQ context: tasks cannot observe a half-written command
       (readers snapshot under a critical section, see MiniPC_Get_CV_Cmd) */
    MiniPC_CV_Cmd.mode              = p->mode;
    MiniPC_CV_Cmd.is_self_color_red = p->is_self_color_red;
    MiniPC_CV_Cmd.yaw               = p->yaw;
    MiniPC_CV_Cmd.yaw_vel           = p->yaw_vel;
    MiniPC_CV_Cmd.yaw_acc           = p->yaw_acc;
    MiniPC_CV_Cmd.pitch             = p->pitch;
    MiniPC_CV_Cmd.pitch_vel         = p->pitch_vel;
    MiniPC_CV_Cmd.pitch_acc         = p->pitch_acc;
    MiniPC_CV_Cmd.forward_vel       = p->forward_vel;
    MiniPC_CV_Cmd.leftward_vel      = p->leftward_vel;
    MiniPC_CV_Cmd.spintop_level     = p->spintop_level;

    MiniPC_Rx_Valid_Count++;
    Rx_last_tick = HAL_GetTick();
}

/**
  * @brief  Feed raw USB CDC bytes into the stream parser.
  * @note   Called from CDC_Receive_HS (USB IRQ). Each Jetson write is one
  *         35-byte transfer, so the common case is a single aligned packet;
  *         the accumulator only works when boundaries get lost.
  */
void MiniPC_Receive_Info(uint8_t *Buff, uint32_t Len)
{
    while (Len > 0)
    {
        /* top up the accumulator */
        uint16_t space = sizeof(Rx_stream) - Rx_stream_len;
        uint16_t n = (Len < space) ? (uint16_t)Len : space;
        memcpy(&Rx_stream[Rx_stream_len], Buff, n);
        Rx_stream_len += n;
        Buff += n;
        Len  -= n;

        /* parse every complete window */
        while (Rx_stream_len >= NUM_BYTES_RX_MINIPC)
        {
            VisionToGimbal_t pkt;
            memcpy(&pkt, Rx_stream, NUM_BYTES_RX_MINIPC);

            if (MiniPC_Rx_Valid(&pkt)) {
                MiniPC_Rx_Accept(&pkt);
                Rx_stream_len -= NUM_BYTES_RX_MINIPC;
                memmove(Rx_stream, &Rx_stream[NUM_BYTES_RX_MINIPC], Rx_stream_len);
            } else {
                /* misaligned: drop one byte and retry */
                MiniPC_Rx_Resync_Count++;
                Rx_stream_len--;
                memmove(Rx_stream, &Rx_stream[1], Rx_stream_len);
            }
        }
    }
}

/* ============================================================
   ACCESSORS (task context)
   ============================================================ */

uint8_t MiniPC_CV_Fresh(void)
{
    return (HAL_GetTick() - Rx_last_tick) < MINIPC_CV_TIMEOUT_MS;
}

uint8_t MiniPC_CV_Fire(void)
{
    return (MiniPC_CV_Cmd.mode == 2) &&
           ((HAL_GetTick() - Rx_last_tick) < MINIPC_CV_FIRE_TIMEOUT_MS);
}

/**
  * @brief  Take an IRQ-safe snapshot of the last CV command.
  * @param  out: filled with the last valid command
  * @retval 1 if the command is fresh AND requests gimbal control (mode >= 1)
  */
uint8_t MiniPC_Get_CV_Cmd(cv_command_t *out)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *out = MiniPC_CV_Cmd;
    uint8_t fresh = MiniPC_CV_Fresh();
    __set_PRIMASK(primask);

    return fresh && (out->mode >= 1);
}
