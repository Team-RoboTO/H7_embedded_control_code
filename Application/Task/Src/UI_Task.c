/**
 * @file    UI_Task.c
 * @brief   HUD interface for the operator (Referee System Client UI).
 */

#include "UI_Task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "CRC.h"
#include "remote_control.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "Chassis_control.h"
#include "INS_Task.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

/* ============================================================================
 * EXTERNAL STATE
 * ============================================================================ */
extern Referee_System_Info_TypeDef Referee_System_Info;
extern NDJ6_Info_Typedef            NDJ6_info;
extern DM_Motor_Info_Typedef        DM_Yaw_Motor;
extern CM_Motor_Info_Typedef        CM_Pitch_Motor;
extern INS_Info_Typedef             INS_Info;

/* Provided by chassis / aim / state machine layers */
extern bool is_rotating;

/* ============================================================================
 * GLOBAL UI STATE (driven by other tasks)
 * ============================================================================ */
int      aimbot_mode             = 0;
int      supercap_dash           = 0;
int      gear_speed_curr_gear    = 1;
uint8_t  charging_state          = 50;
uint16_t g_motor_fault           = 0;

typedef enum {
    FEEDER_STANDBY,
    FEEDER_SPINUP,
    FEEDER_LOADED,
    FEEDER_JAM,
    FEEDER_OVERHEAT,
    FEEDER_STEP,
    FEEDER_FIRING
} feeder_state_e;

feeder_state_e feeder_state = FEEDER_STANDBY;

/* ============================================================================
 * INTERNAL STATE
 * ============================================================================ */
static uint16_t g_client_id   = 0;
static uint8_t  g_ref_tx_seq  = 0;

/* Top-bar X coordinates (computed once in init) */
static uint32_t spin_coords     = 0;
static uint32_t gear_coords     = 0;
static uint32_t aimbot_coords   = 0;
static uint32_t supercap_coords = 0;

/* Previous states (for change detection) */
static int prev_spinspin      = -1;
static int prev_spin_warning  = -1;
static int prev_aimbot        = -1;
static int prev_supercap_dash = -1;
static int prev_gear          = -1;
static int prev_motor_error   = -1;
static int prev_feeder_state  = -1;
static int motor_fault_enabled  = 0;
static int feeder_state_enabled = 0;

/* TX synchronization: semaphore released on DMA TxCplt */
static osSemaphoreId ui_tx_done_sem  = NULL;  /* signaled when DMA completes */
static osSemaphoreId ui_send_mtx     = NULL;  /* protects ref_send / seq */

/* DMA TX buffer: must be in AXI_SRAM, 32-byte aligned for D-Cache */
__attribute__((section(".AXI_SRAM"), aligned(32)))
static uint8_t ui_tx_dma_buffer[512];

volatile uint32_t ui_tx_count = 0;  /* debug counter */
volatile uint32_t tx_cplt_count = 0;
volatile uint32_t tx_start_count = 0;

int g_spinspin_mode = 0;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        tx_cplt_count++;
        UI_NotifyTxComplete();
    }
}

/* ============================================================================
 * LOW-LEVEL: ROBOT ID -> CLIENT ID MAPPING
 * ============================================================================ */
static void map_robot_id(uint16_t robot_id)
{
    if (robot_id == 0)               g_client_id = 0;
    else if (robot_id < 100)         g_client_id = 0x0100 + robot_id;
    else                             g_client_id = 0x0164 + (robot_id - 100);
}

/* ============================================================================
 * LOW-LEVEL: TX COMPLETION NOTIFICATION
 * ============================================================================ */
void UI_NotifyTxComplete(void)
{
    if (ui_tx_done_sem != NULL) {
        osSemaphoreRelease(ui_tx_done_sem);
    }
}

/* ============================================================================
 * LOW-LEVEL: UNIFIED SEND PATH
 * ============================================================================ */
static void ref_send(uint8_t *tx_buffer, uint16_t tx_len)
{
    uint16_t total_len = tx_len + 2;  /* +2 for trailing CRC16 */

    if (total_len > sizeof(ui_tx_dma_buffer)) return;
    if (g_client_id == 0) return;     /* no client to talk to yet */

    /* Serialize all senders */
    osSemaphoreWait(ui_send_mtx, osWaitForever);

    /* Generate sequences and CRCs safely inside the mutex lock */
    ref_frame_header_t *hdr = (ref_frame_header_t *)tx_buffer;
    hdr->seq = g_ref_tx_seq++;
    Append_CRC8_Check_Sum(tx_buffer, 5);
    Append_CRC16_Check_Sum(tx_buffer, total_len);

    /* Recover UART from error state if needed */
    if (huart1.gState == HAL_UART_STATE_ERROR) {
        HAL_UART_AbortTransmit(&huart1);
        huart1.gState = HAL_UART_STATE_READY;
    }

    /* Drain any stale completion signal so we wait for THIS transfer */
    while (osSemaphoreWait(ui_tx_done_sem, 0) == osOK) { /* drain */ }

    /* Copy into AXI_SRAM DMA buffer */
    memcpy(ui_tx_dma_buffer, tx_buffer, total_len);

    /* D-Cache clean, rounded up to full 32-byte cache lines */
    uint32_t clean_len = (total_len + 31U) & ~31U;
    SCB_CleanDCache_by_Addr((uint32_t *)ui_tx_dma_buffer, clean_len);

    if (HAL_UART_Transmit_DMA(&huart1, ui_tx_dma_buffer, total_len) == HAL_OK) {
        /* Wait for DMA TxCplt (with safety timeout) */
        if (osSemaphoreWait(ui_tx_done_sem, 100) != osOK) {
            HAL_UART_AbortTransmit(&huart1);
            huart1.gState = HAL_UART_STATE_READY;
        }
        ui_tx_count++;
    } else {
        huart1.gState = HAL_UART_STATE_READY;
    }

    osSemaphoreRelease(ui_send_mtx);

    /* Referee 10Hz/cmd_id throttle (only matters when many calls run back-to-back) */
    osDelay(REF_DELAY);
}

/* ============================================================================
 * LOW-LEVEL: HEADER BUILDERS
 * ============================================================================ */
static uint16_t build_graphic_header(uint8_t *tx_buffer, uint8_t num_graphics)
{
    uint16_t sub_cmd_id;
    if      (num_graphics == 1) sub_cmd_id = 0x0101;
    else if (num_graphics == 2) sub_cmd_id = 0x0102;
    else if (num_graphics == 5) sub_cmd_id = 0x0103;
    else if (num_graphics == 7) sub_cmd_id = 0x0104;
    else                        return 0;  /* invalid count, refuse to build */

    ref_frame_header_t *hdr = (ref_frame_header_t *)tx_buffer;
    hdr->start_frame = 0xA5;
    hdr->data_length = sizeof(ref_inter_robot_data_t)
                     + sizeof(graphic_data_struct_t) * num_graphics;
    hdr->cmd_id = 0x0301;

    ref_inter_robot_data_t *gh = (ref_inter_robot_data_t *)(tx_buffer + sizeof(ref_frame_header_t));
    gh->cmd_ID      = sub_cmd_id;
    gh->send_ID     = Referee_System_Info.robot_status.robot_id;
    gh->receiver_ID = g_client_id;
    return sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t);
}

static uint16_t build_char_header(uint8_t *tx_buffer)
{
    ref_frame_header_t *hdr = (ref_frame_header_t *)tx_buffer;
    hdr->start_frame = 0xA5;
    hdr->data_length = sizeof(ref_inter_robot_data_t)
                     + sizeof(graphic_data_struct_t) + 30;
    hdr->cmd_id = 0x0301;

    ref_inter_robot_data_t *gh = (ref_inter_robot_data_t *)(tx_buffer + sizeof(ref_frame_header_t));
    gh->cmd_ID      = 0x0110;
    gh->send_ID     = Referee_System_Info.robot_status.robot_id;
    gh->receiver_ID = g_client_id;
    return sizeof(ref_frame_header_t) + sizeof(ref_inter_robot_data_t);
}

/* ============================================================================
 * HELPER: SET NAME / OPERATION / TYPE
 * ============================================================================ */
static void set_name(graphic_data_struct_t *g, char a, char b, char c)
{
    g->graphic_name[0] = a;
    g->graphic_name[1] = b;
    g->graphic_name[2] = c;
}

/* ============================================================================
 * CLEAR / DELETE HELPERS
 * ============================================================================ */
static void clear_hud(void)
{
    uint8_t tx_buffer[64];
    memset(tx_buffer, 0, sizeof(tx_buffer));

    ref_frame_header_t *hdr = (ref_frame_header_t *)tx_buffer;
    hdr->start_frame = 0xA5;
    hdr->data_length = sizeof(ref_delete_graphic_t);
    hdr->cmd_id = 0x0301;

    ref_delete_graphic_t *del = (ref_delete_graphic_t *)(tx_buffer + sizeof(ref_frame_header_t));
    del->cmd_ID             = 0x0100;
    del->send_ID            = Referee_System_Info.robot_status.robot_id;
    del->receiver_ID        = g_client_id;
    del->graphic_operation  = 2;  /* delete all */
    del->graphic_layer      = 9;

    ref_send(tx_buffer, sizeof(ref_frame_header_t) + sizeof(ref_delete_graphic_t));
}

static void delete_layer(uint8_t layer)
{
    uint8_t tx_buffer[64];
    memset(tx_buffer, 0, sizeof(tx_buffer));

    ref_frame_header_t *hdr = (ref_frame_header_t *)tx_buffer;
    hdr->start_frame = 0xA5;
    hdr->data_length = sizeof(ref_delete_graphic_t);
    hdr->cmd_id = 0x0301;

    ref_delete_graphic_t *del = (ref_delete_graphic_t *)(tx_buffer + sizeof(ref_frame_header_t));
    del->cmd_ID             = 0x0100;
    del->send_ID            = Referee_System_Info.robot_status.robot_id;
    del->receiver_ID        = g_client_id;
    del->graphic_operation  = 1;  /* delete layer */
    del->graphic_layer      = layer;

    ref_send(tx_buffer, sizeof(ref_frame_header_t) + sizeof(ref_delete_graphic_t));
}

/* ============================================================================
 * COORDINATE LAYOUT
 * ============================================================================ */
static void set_top_coordinates(void)
{
    /* 4 elements: SPIN | GEAR | AIM | CAP */
    spin_coords     = CENTER_X - (uint32_t)(TOP_GAP * 1.5f);
    gear_coords     = CENTER_X - (TOP_GAP / 2);
    aimbot_coords   = CENTER_X + (TOP_GAP / 2);
    supercap_coords = CENTER_X + (uint32_t)(TOP_GAP * 1.5f);
}

/* ============================================================================
 * CHAR-DRAWING HELPER (one text graphic per call)
 * ============================================================================ */
static void send_char_graphic(const char *text, uint8_t name_a, uint8_t name_b, uint8_t name_c,
                              uint8_t layer, uint8_t color, uint8_t font_size, uint8_t width,
                              uint32_t x, uint32_t y, uint8_t op)
{
    uint8_t tx_buffer[128];
    char    char_buffer[30];
    
    memset(tx_buffer, 0, sizeof(tx_buffer));
    memset(char_buffer, 0, sizeof(char_buffer));

    uint8_t char_len = (uint8_t)snprintf(char_buffer, 30, "%s", text);

    uint16_t pos = build_char_header(tx_buffer);

    graphic_data_struct_t *g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, name_a, name_b, name_c);
    g->layer          = layer;
    g->color          = color;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_CHAR;
    g->details_a      = font_size;
    g->details_b      = char_len;
    g->width          = width;
    g->start_x        = x;
    g->start_y        = y;
    pos += sizeof(graphic_data_struct_t);

    memcpy(tx_buffer + pos, char_buffer, 30);
    pos += 30;

    ref_send(tx_buffer, pos);
}

/* ============================================================================
 * TOP BAR: SPIN / GEAR / AIM / CAP TEXT
 * ============================================================================ */
static void draw_spin_text(uint8_t op)
{
    uint32_t x = spin_coords - CHAR_X_OFFSET * (is_rotating ? 2 : 3);
    send_char_graphic(is_rotating ? "ON" : "OFF",
                      'C', 'H', 'A', 0,
                      is_rotating ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE,
                      FONT_SIZE, CHAR_WIDTH,
                      x, TOP_Y_POS + CHAR_Y_OFFSET, op);
}

static void draw_gear_text(uint8_t op)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "GEAR %d", gear_speed_curr_gear);
    uint8_t len = (uint8_t)strlen(buf);
    send_char_graphic(buf, 'G', 'E', 'A', 5, GRAPHIC_COLOUR_CYAN,
                      FONT_SIZE, CHAR_WIDTH,
                      gear_coords - CHAR_X_OFFSET * len,
                      TOP_Y_POS + CHAR_Y_OFFSET, op);
}

static void draw_aim_text(uint8_t op)
{
    const char *txt = aimbot_mode ? "AIM ON" : "AIM OFF";
    uint8_t len = (uint8_t)strlen(txt);
    send_char_graphic(txt, 'A', 'I', 'M', 4,
                      aimbot_mode ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE,
                      FONT_SIZE, CHAR_WIDTH,
                      aimbot_coords - CHAR_X_OFFSET * len,
                      TOP_Y_POS + CHAR_Y_OFFSET, op);
}

static void draw_supercap_text(uint8_t op)
{
    const char *txt = supercap_dash ? "CAP ON" : "CAP OFF";
    uint8_t len = (uint8_t)strlen(txt);
    send_char_graphic(txt, 'C', 'A', 'P', 4,
                      supercap_dash ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE,
                      FONT_SIZE, CHAR_WIDTH,
                      supercap_coords - CHAR_X_OFFSET * len,
                      TOP_Y_POS + CHAR_Y_OFFSET, op);
}

/* ============================================================================
 * DYNAMIC: SPIN BORDER ARC + SUPERCAP ARC + CURRENT PITCH + BULLET BAR
 * (all packed into one multi-graphic packet)
 * ============================================================================ */
static void fill_spin_border(graphic_data_struct_t *g, uint8_t op)
{
    set_name(g, 'B', 'O', 'R');
    g->layer          = 2;
    g->color          = is_rotating ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_ARC;

    float chassis_dir = DM_Yaw_Motor.Data.Angle * 57.2958f;
    int   base = (chassis_dir < 0) ? (int)(360 + chassis_dir) : (int)chassis_dir;
    g->details_a = base + BORDER_GAP_SIZE;
    g->details_b = base - BORDER_GAP_SIZE;

    g->width     = 7;
    g->start_x   = spin_coords;
    g->start_y   = TOP_Y_POS;
    g->details_d = 50;
    g->details_e = 50;
}

static void fill_supercap_arc(graphic_data_struct_t *g, uint8_t op)
{
    set_name(g, 'S', 'U', 'P');
    g->layer          = 3;
    g->color          = (charging_state > SUPERCAP_ENABLE_THRESHOLD)
                        ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_ARC;
    g->details_a      = 270;

    int   range  = 100 - SUPERCAP_DISABLE_THRESHOLD;
    float mapped = fmaxf(0.0f, fminf(1.0f,
                       (float)(charging_state - SUPERCAP_DISABLE_THRESHOLD) / (float)range));
    int   level  = (int)fmaxf(1.0f, mapped * ANGLE_LIMIT);
    g->details_b = 270 + level;

    g->width     = 30;
    g->start_x   = CENTER_X;
    g->start_y   = CENTER_Y;
    g->details_d = RADIAL_DIAMETER;
    g->details_e = RADIAL_DIAMETER;
}

static void fill_curr_pitch(graphic_data_struct_t *g, uint8_t op)
{
    set_name(g, 'P', 'I', 'T');
    g->layer          = 1;
    g->color          = PITCH_ANG_COLOUR;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_LINE;
    g->width          = PITCH_ANG_WIDTH;

    float curr_ang = PITCH_INVERT * INS_Info.Roll_Angle * (ANGLE_LIMIT / PITCH_RANGE_DEG);
    uint32_t xpos = CENTER_X + (int)(RADIAL_DIAMETER * cosf(curr_ang * 0.0174533f));
    uint32_t ypos = CENTER_Y + (int)(RADIAL_DIAMETER * sinf(curr_ang * 0.0174533f));

    g->start_x   = xpos - MAJOR_TICK_LENGTH / 2;
    g->start_y   = ypos;
    g->details_d = xpos + MAJOR_TICK_LENGTH / 2;
    g->details_e = ypos;
}

static void fill_bullet_bar_border(graphic_data_struct_t *g, uint8_t op)
{
    set_name(g, 'B', 'B', 'O');
    g->layer          = 3;
    g->color          = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_RECTANGLE;
    g->width          = 3;
    g->start_x        = 1890 - 15;
    g->start_y        = 350;
    g->details_d      = 1890 + 15;
    g->details_e      = 350 + 350;
}

static void fill_bullet_bar_level(graphic_data_struct_t *g, uint8_t op)
{
    uint16_t bullets = Referee_System_Info.projectile_allowance.projectile_allowance_17mm;
    if (bullets > 200) bullets = 200;
    uint32_t bar_h = (bullets * 350) / 200;

    uint8_t color = GRAPHIC_COLOUR_GREEN;
    if      (bullets < 50)  color = GRAPHIC_COLOUR_PURPLISH_RED;
    else if (bullets < 150) color = GRAPHIC_COLOUR_YELLOW;

    set_name(g, 'B', 'B', 'L');
    g->layer          = 3;
    g->color          = color;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_LINE;
    g->width          = 20;
    g->start_x        = 1890;
    g->start_y        = 351;
    g->details_d      = 1890;
    g->details_e      = 351 + bar_h;
}

/* ============================================================================
 * ITALIAN FLAG + SPQR LABEL (above the bullet bar, static)
 *========================================================================== */
#define FLAG_X_LEFT   1820u
#define FLAG_X_MID    1840u
#define FLAG_X_MID2   1860u
#define FLAG_X_RIGHT  1880u
#define FLAG_Y_BOT    750u
#define FLAG_Y_TOP    800u
#define FLAG_LAYER    5

static void draw_italy_flag_spqr(uint8_t op)
{
    uint8_t  tx_buffer[256];
    uint16_t pos;
    graphic_data_struct_t *g;

    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 2);
    if (pos == 0) return;

    /* Striscia Verde */
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'I', 'T', 'G');
    g->layer          = FLAG_LAYER;
    g->color          = GRAPHIC_COLOUR_GREEN;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_LINE;   // Modificato in LINE
    g->width          = 20;                  // Lo spessore riempie i 20 pixel
    g->start_x        = 1830u;               // Centro della striscia
    g->start_y        = FLAG_Y_BOT;
    g->details_d      = 1830u;               // Centro della striscia
    g->details_e      = FLAG_Y_TOP;
    pos += sizeof(graphic_data_struct_t);

    /* Striscia Bianca */
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'I', 'T', 'W');
    g->layer          = FLAG_LAYER;
    g->color          = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_LINE;   // Modificato in LINE
    g->width          = 20;
    g->start_x        = 1850u;               // Centro della striscia
    g->start_y        = FLAG_Y_BOT;
    g->details_d      = 1850u;               // Centro della striscia
    g->details_e      = FLAG_Y_TOP;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    /* Pacchetto 2: Striscia Rossa + Testo SPQR */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 1);
    if (pos == 0) return;

    /* Striscia Rossa */
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'I', 'T', 'R');
    g->layer          = FLAG_LAYER;
    g->color          = GRAPHIC_COLOUR_OWN_COLOR; 
    g->operation_type = op;
    g->graphic_type   = GRAPHIC_TYPE_LINE;   // Modificato in LINE
    g->width          = 20;
    g->start_x        = 1870u;               // Centro della striscia
    g->start_y        = FLAG_Y_BOT;
    g->details_d      = 1870u;               // Centro della striscia
    g->details_e      = FLAG_Y_TOP;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    send_char_graphic("DIOFA",
                      'S', 'P', 'Q',
                      FLAG_LAYER,
                      GRAPHIC_COLOUR_WHITE,
                      FONT_SIZE, CHAR_WIDTH, 
                      FLAG_X_LEFT - 5,
                      FLAG_Y_BOT - 20,       
                      op);
}

static void draw_dynamic(uint8_t op)
{
    uint8_t tx_buffer[256];
    memset(tx_buffer, 0, sizeof(tx_buffer));
    
    uint16_t pos = build_graphic_header(tx_buffer, 5);
    if (pos == 0) return;

    fill_spin_border       ((graphic_data_struct_t *)(tx_buffer + pos), op); pos += sizeof(graphic_data_struct_t);
    fill_supercap_arc      ((graphic_data_struct_t *)(tx_buffer + pos), op); pos += sizeof(graphic_data_struct_t);
    fill_curr_pitch        ((graphic_data_struct_t *)(tx_buffer + pos), op); pos += sizeof(graphic_data_struct_t);
    fill_bullet_bar_border ((graphic_data_struct_t *)(tx_buffer + pos), op); pos += sizeof(graphic_data_struct_t);
    fill_bullet_bar_level  ((graphic_data_struct_t *)(tx_buffer + pos), op); pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);
}

/* ============================================================================
 * CROSSHAIR (static, sent once on connect)
 * ============================================================================ */
static void draw_crosshair(uint8_t op)
{
    uint8_t  tx_buffer[256];
    uint16_t pos;
    graphic_data_struct_t *g;

    /* ============================================================================
     * PACKET 1: THE CORE RETICLE (7 graphics)
     * ============================================================================ */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 7);
    if (pos == 0) return;

    // 1. Circle Outer (Sleek cyan scope ring)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'C', 'O', 'R'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_CIRCLE; g->width = 1;
    g->start_x = CENTER_X; g->start_y = CENTER_Y;
    g->details_c = 120;
    pos += sizeof(graphic_data_struct_t);

    // 2. Circle Inner (Small focal ring)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'C', 'I', 'N'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_CIRCLE; g->width = 1;
    g->start_x = CENTER_X; g->start_y = CENTER_Y;
    g->details_c = 30;
    pos += sizeof(graphic_data_struct_t);

    // 3. Center Dot (Lethal red laser dot)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'D', 'O', 'T'); g->layer = 0; g->color = GRAPHIC_COLOUR_PURPLISH_RED;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_CIRCLE; g->width = 4;
    g->start_x = CENTER_X; g->start_y = CENTER_Y;
    g->details_c = 2;
    pos += sizeof(graphic_data_struct_t);

    // 4. Horizontal Left Crosshair Bar
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'H', 'O', 'L'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 180; g->start_y = CENTER_Y;
    g->details_d = CENTER_X - 45;  g->details_e = CENTER_Y;
    pos += sizeof(graphic_data_struct_t);

    // 5. Horizontal Right Crosshair Bar
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'H', 'O', 'R'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X + 45;  g->start_y = CENTER_Y;
    g->details_d = CENTER_X + 180; g->details_e = CENTER_Y;
    pos += sizeof(graphic_data_struct_t);

    // 6. Vertical Top Crosshair Bar
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'V', 'E', 'T'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X;        g->start_y = CENTER_Y + 45;
    g->details_d = CENTER_X;      g->details_e = CENTER_Y + 180;
    pos += sizeof(graphic_data_struct_t);

    // 7. Vertical Bottom Crosshair Bar (Full BDC drop line extending to bottom)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'V', 'E', 'B'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X;        g->start_y = CENTER_Y - 45;
    g->details_d = CENTER_X;      g->details_e = 50; // all the way to bottom (y=50)
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    /* ============================================================================
     * PACKET 2: TACTICAL BRACKETS PART 1 (7 graphics)
     * ============================================================================ */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 7);
    if (pos == 0) return;

    // Top-Left Corner Bracket (Horizontal & Vertical)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'L', 'H'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 240; g->start_y = CENTER_Y + 200;
    g->details_d = CENTER_X - 200; g->details_e = CENTER_Y + 200;
    pos += sizeof(graphic_data_struct_t);

    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'L', 'V'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 240; g->start_y = CENTER_Y + 200;
    g->details_d = CENTER_X - 240; g->details_e = CENTER_Y + 160;
    pos += sizeof(graphic_data_struct_t);

    // Top-Right Corner Bracket (Horizontal & Vertical)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'R', 'H'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X + 240; g->start_y = CENTER_Y + 200;
    g->details_d = CENTER_X + 200; g->details_e = CENTER_Y + 200;
    pos += sizeof(graphic_data_struct_t);

    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'R', 'V'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X + 240; g->start_y = CENTER_Y + 200;
    g->details_d = CENTER_X + 240; g->details_e = CENTER_Y + 160;
    pos += sizeof(graphic_data_struct_t);

    // Bottom-Left Corner Bracket (Horizontal & Vertical)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'B', 'L', 'H'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 240; g->start_y = CENTER_Y - 200;
    g->details_d = CENTER_X - 200; g->details_e = CENTER_Y - 200;
    pos += sizeof(graphic_data_struct_t);

    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'B', 'L', 'V'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 240; g->start_y = CENTER_Y - 200;
    g->details_d = CENTER_X - 240; g->details_e = CENTER_Y - 160;
    pos += sizeof(graphic_data_struct_t);

    // Bottom-Right Corner Bracket (Horizontal part)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'B', 'R', 'H'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X + 240; g->start_y = CENTER_Y - 200;
    g->details_d = CENTER_X + 200; g->details_e = CENTER_Y - 200;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    /* ============================================================================
     * PACKET 3: BR BRACKET + TOP BDC TICKS (5 graphics)
     * ============================================================================ */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 5);
    if (pos == 0) return;

    // Bottom-Right Corner Bracket (Vertical part)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'B', 'R', 'V'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X + 240; g->start_y = CENTER_Y - 200;
    g->details_d = CENTER_X + 240; g->details_e = CENTER_Y - 160;
    pos += sizeof(graphic_data_struct_t);

    // BDC Ticks 1 to 4 (Spanning from center downwards)
    // Tick 1
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '1'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 40;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 40;
    pos += sizeof(graphic_data_struct_t);

    // Tick 2
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '2'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 80;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 80;
    pos += sizeof(graphic_data_struct_t);

    // Tick 3
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '3'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 120;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 120;
    pos += sizeof(graphic_data_struct_t);

    // Tick 4 (Major Tick, wider)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '4'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 12;   g->start_y = CENTER_Y - 40 - 160;
    g->details_d = CENTER_X + 12; g->details_e = CENTER_Y - 40 - 160;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    /* ============================================================================
     * PACKET 4: REMAINING BDC TICKS (7 graphics)
     * ============================================================================ */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 7);
    if (pos == 0) return;

    // Tick 5
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '5'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 200;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 200;
    pos += sizeof(graphic_data_struct_t);

    // Tick 6
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '6'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 240;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 240;
    pos += sizeof(graphic_data_struct_t);

    // Tick 7
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '7'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 280;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 280;
    pos += sizeof(graphic_data_struct_t);

    // Tick 8 (Major Tick, wider)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '8'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 14;   g->start_y = CENTER_Y - 40 - 320;
    g->details_d = CENTER_X + 14; g->details_e = CENTER_Y - 40 - 320;
    pos += sizeof(graphic_data_struct_t);

    // Tick 9
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', '9'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 360;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 360;
    pos += sizeof(graphic_data_struct_t);

    // Tick 10 (Tick A)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', 'A'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 7;    g->start_y = CENTER_Y - 40 - 400;
    g->details_d = CENTER_X + 7;  g->details_e = CENTER_Y - 40 - 400;
    pos += sizeof(graphic_data_struct_t);

    // Tick 11 (Tick B, Major Tick near bottom)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'K', 'B'); g->layer = 0; g->color = GRAPHIC_COLOUR_WHITE;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 2;
    g->start_x = CENTER_X - 16;   g->start_y = CENTER_Y - 40 - 440;
    g->details_d = CENTER_X + 16; g->details_e = CENTER_Y - 40 - 440;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);

    /* ============================================================================
     * PACKET 5: CHASSIS ORIENTATION GUIDE LINES (2 graphics)
     * ============================================================================ */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 2);
    if (pos == 0) return;

    // Track Left Guide Line (Sleek cyan line)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'R', 'L'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X - 40;  g->start_y = CENTER_Y - 80;
    g->details_d = CENTER_X - 350; g->details_e = CENTER_Y - 540;
    pos += sizeof(graphic_data_struct_t);

    // Track Right Guide Line (Sleek cyan line)
    g = (graphic_data_struct_t *)(tx_buffer + pos);
    set_name(g, 'T', 'R', 'R'); g->layer = 0; g->color = GRAPHIC_COLOUR_CYAN;
    g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE; g->width = 1;
    g->start_x = CENTER_X + 40;  g->start_y = CENTER_Y - 80;
    g->details_d = CENTER_X + 350; g->details_e = CENTER_Y - 540;
    pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, pos);
}

/* ============================================================================
 * PITCH SCALE (static)
 * ============================================================================ */
static void draw_pitch_ticks(uint8_t op)
{
    uint8_t  tx_buffer[256];
    uint16_t pos;
    graphic_data_struct_t *g;

    /* Major ticks (5) */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 5);
    if (pos == 0) return;
    float angles_maj[5] = { ANGLE_LIMIT, ANGLE_LIMIT/2.0f, 0.0f, -ANGLE_LIMIT/2.0f, -ANGLE_LIMIT };
    for (int i = 0; i < 5; i++) {
        float a = angles_maj[i] * 0.0174533f;
        uint32_t x = CENTER_X + (int)(RADIAL_DIAMETER * cosf(a));
        uint32_t y = CENTER_Y + (int)(RADIAL_DIAMETER * sinf(a));
        g = (graphic_data_struct_t *)(tx_buffer + pos);
        set_name(g, 'M', 'A', (uint8_t)(i + 1));
        g->layer = 1; g->color = TICK_COLOUR;
        g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE;
        g->width = MAJOR_TICK_WIDTH;
        g->start_x = x - MAJOR_TICK_LENGTH/2; g->start_y = y;
        g->details_d = x + MAJOR_TICK_LENGTH/2; g->details_e = y;
        pos += sizeof(graphic_data_struct_t);
    }
    ref_send(tx_buffer, pos);

    /* Minor ticks (5 graphics) */
    memset(tx_buffer, 0, sizeof(tx_buffer));
    pos = build_graphic_header(tx_buffer, 5);
    if (pos == 0) return;
    float gap = ANGLE_LIMIT / 4.0f;
    float angles_min[5] = { gap * 3.0f, gap, 0.0f, -gap, -gap * 3.0f };
    for (int i = 0; i < 5; i++) {
        float a = angles_min[i] * 0.0174533f;
        uint32_t x = CENTER_X + (int)(RADIAL_DIAMETER * cosf(a));
        uint32_t y = CENTER_Y + (int)(RADIAL_DIAMETER * sinf(a));
        g = (graphic_data_struct_t *)(tx_buffer + pos);
        set_name(g, 'M', 'I', (uint8_t)(i + 1));
        g->layer = 1; g->color = TICK_COLOUR;
        g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE;
        g->width = MINOR_TICK_WIDTH;
        g->start_x = x - MINOR_TICK_LENGTH/2; g->start_y = y;
        g->details_d = x + MINOR_TICK_LENGTH/2; g->details_e = y;
        pos += sizeof(graphic_data_struct_t);
    }
    ref_send(tx_buffer, pos);

    /* Pitch labels (one CHAR per send, 5 total) */
    int tick_num = 2;
    for (int i = 0; i < 5; i++) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", tick_num * TICK_INTERVALS * 2);
        tick_num--;
        float a = angles_maj[i] * 0.0174533f;
        uint32_t x = CENTER_X + (int)(RADIAL_DIAMETER * cosf(a));
        uint32_t y = CENTER_Y + (int)(RADIAL_DIAMETER * sinf(a));
        send_char_graphic(buf, 'L', 'A', (uint8_t)(i + 1), 1, TICK_COLOUR,
                          FONT_SIZE_SMALL, CHAR_WIDTH_SMALL,
                          x + PITCH_LABEL_DIST, y + CHAR_Y_OFFSET, op);
    }
}

static void draw_pitch_limits(uint8_t op)
{
    uint8_t  tx_buffer[128];
    uint16_t pos;
    graphic_data_struct_t *g;
    
    memset(tx_buffer, 0, sizeof(tx_buffer));

    float graphic_edge = TICK_INTERVALS * 4 * 0.0174533f;
    float max_ang = -PITCH_INVERT * INS_Info.Roll_Angle * ANGLE_LIMIT / graphic_edge;
    float min_ang = -PITCH_INVERT * -INS_Info.Roll_Angle * ANGLE_LIMIT / graphic_edge;
    float angs[2] = { max_ang, min_ang };

    pos = build_graphic_header(tx_buffer, 2);
    if (pos == 0) return;
    for (int i = 0; i < 2; i++) {
        float a = angs[i] * 0.0174533f;
        uint32_t x = CENTER_X + (int)(RADIAL_DIAMETER * cosf(a));
        uint32_t y = CENTER_Y + (int)(RADIAL_DIAMETER * sinf(a));
        g = (graphic_data_struct_t *)(tx_buffer + pos);
        set_name(g, 'L', 'I', (uint8_t)(i + 1));
        g->layer = 1; g->color = PITCH_BOUNDARY_COLOUR;
        g->operation_type = op; g->graphic_type = GRAPHIC_TYPE_LINE;
        g->width = PITCH_BOUNDARY_WIDTH;
        g->start_x = x - MAJOR_TICK_LENGTH/2; g->start_y = y;
        g->details_d = x + MAJOR_TICK_LENGTH/2; g->details_e = y;
        pos += sizeof(graphic_data_struct_t);
    }
    ref_send(tx_buffer, pos);
}

/* ============================================================================
 * ALARMS: SPIN WARNING, MOTOR FAULT, FEEDER STATE
 * ============================================================================ */

uint8_t  spin_warning_color = 0;
uint8_t  change_color = 0;
static TickType_t last_blink_tick = 0;

static void draw_spin_warning_op(uint8_t op)
{
    send_char_graphic("RUOTAAA!", 'W', 'R', 'N', 9, spin_warning_color,
                      100, 5,
                      CENTER_X - (42 * 8),
                      CENTER_Y + 310, op);
}

static void update_spin_warning(void)
{
    int current = !is_rotating;

    if (current) {
        if ((xTaskGetTickCount() - last_blink_tick) >= pdMS_TO_TICKS(200)) {
            last_blink_tick = xTaskGetTickCount();
            change_color += 1;
            spin_warning_color = (change_color % 2) ? 4 : 8;

            if (prev_spin_warning != 1) draw_spin_warning_op(GRAPHIC_ADD);
            else                        draw_spin_warning_op(GRAPHIC_MODIFY);
        }
    }
    else {
        if (prev_spin_warning == 1) draw_spin_warning_op(GRAPHIC_DELETE);
    }

    prev_spin_warning = current;
}

static void draw_motor_fault_line(const char *label, uint8_t na, uint8_t nb, uint8_t nc,
                                  uint8_t row, uint8_t op)
{
    send_char_graphic(label, na, nb, nc, 7, GRAPHIC_COLOUR_PURPLISH_RED,
                      FONT_SIZE, CHAR_WIDTH,
                      50, MOTOR_FAULT_START - MOTOR_FAULT_GAP * row, op);
}

static void draw_motor_fault(uint8_t op)
{
    char buf[30];
    uint8_t row = 0;

    buf[0] = 0;
    for (int i = 0; i < 4; i++) {
        if (g_motor_fault & (1u << i)) {
            if (buf[0]) strncat(buf, ", ", sizeof(buf) - strlen(buf) - 1);
            switch (i) {
                case 0: strncat(buf, "FR", sizeof(buf) - strlen(buf) - 1); break;
                case 1: strncat(buf, "FL", sizeof(buf) - strlen(buf) - 1); break;
                case 2: strncat(buf, "BL", sizeof(buf) - strlen(buf) - 1); break;
                case 3: strncat(buf, "BR", sizeof(buf) - strlen(buf) - 1); break;
            }
        }
    }
    if (buf[0]) { draw_motor_fault_line(buf, 'C', 'H', 'S', row++, op); }

    buf[0] = 0;
    for (int i = 4; i < 7; i++) {
        if (g_motor_fault & (1u << i)) {
            if (buf[0]) strncat(buf, ", ", sizeof(buf) - strlen(buf) - 1);
            switch (i) {
                case 4: strncat(buf, "LF",   sizeof(buf) - strlen(buf) - 1); break;
                case 5: strncat(buf, "RF",   sizeof(buf) - strlen(buf) - 1); break;
                case 6: strncat(buf, "FEED", sizeof(buf) - strlen(buf) - 1); break;
            }
        }
    }
    if (buf[0]) { draw_motor_fault_line(buf, 'L', 'A', 'U', row++, op); }

    buf[0] = 0;
    for (int i = 7; i < 9; i++) {
        if (g_motor_fault & (1u << i)) {
            if (buf[0]) strncat(buf, ", ", sizeof(buf) - strlen(buf) - 1);
            switch (i) {
                case 7: strncat(buf, "PITCH", sizeof(buf) - strlen(buf) - 1); break;
                case 8: strncat(buf, "YAW",   sizeof(buf) - strlen(buf) - 1); break;
            }
        }
    }
    if (buf[0]) { draw_motor_fault_line(buf, 'P', 'A', 'Y', row++, op); }
}

static void update_motor_fault(void)
{
    bool in_match = (Referee_System_Info.game_status.game_progress == 4
                  || Referee_System_Info.game_status.game_progress == 3);

    if (in_match) {
        if (motor_fault_enabled) { delete_layer(7); motor_fault_enabled = 0; }
        return;
    }

    if (!motor_fault_enabled) {
        draw_motor_fault(GRAPHIC_ADD);
        motor_fault_enabled = 1;
        prev_motor_error    = g_motor_fault;
    } else if (prev_motor_error != g_motor_fault) {
        delete_layer(7);
        draw_motor_fault(GRAPHIC_ADD);
        prev_motor_error = g_motor_fault;
    }
}

static void draw_feeder(uint8_t op)
{
		// CM Chassis Motors
		for (int i = 0; i < 4; i++) {
				char *txt_CM_motor;
				uint8_t color_CM_chassis;
				switch (CM_Chassis_Motor[i].Data.Error) {
						case CM_NO_ERROR:          txt_CM_motor = "OK";            color_CM_chassis = GRAPHIC_COLOUR_GREEN; break;
						case CM_OVERTEMPERATURE:   txt_CM_motor = "OVERTEMP";      break;
						case CM_OVERCURRENT:       txt_CM_motor = "OVERCURRENT";   break;
						case CM_OVERVOLTAGE:       txt_CM_motor = "OVERVOLTAGE";   break;
						case CM_UNDERVOLTAGE:      txt_CM_motor = "UNDERVOLTAGE";  break;
						case CM_ENCODER_FAULT:     txt_CM_motor = "ENCODER_FAULT"; break;
						case CM_PHASE_UNBALANCE:   txt_CM_motor = "PHASE_UNBAL";   break;
						case CM_NO_COMM:           txt_CM_motor = "NO_COMM";       break;
						default:                   txt_CM_motor = "UNKNOWN";       break;
				}
				
					snprintf(full_txt, sizeof(full_txt), "%s%d:%s\n","CH", CM_Chassis_Motor[i].FDCANFrame.TxIdentifier, txt_CM_motor);
					snprintf(text, sizeof(text), "%s %s",text, full_txt);
//					send_char_graphic(full_txt, 'F', 'D', 'R', 6+i, color_CM_chassis, FONT_SIZE*2/3, CHAR_WIDTH*2/3,
//									50, 800 - i * (FONT_SIZE*2/3 + 5), op);
				
		}
		send_char_graphic(text, 'F', 'D', 'R', 6, GRAPHIC_COLOUR_GREEN, FONT_SIZE*2/3, CHAR_WIDTH*2/3,
                      50, 800, op);
		
		// DM Yaw Motor ...etc
}

static void update_feeder(void)
{
    bool in_match = (Referee_System_Info.game_status.game_progress == 4
                  || Referee_System_Info.game_status.game_progress == 3);

    if (in_match) {
        if (feeder_state_enabled) { delete_layer(6); feeder_state_enabled = 0; }
        return;
    }

    if (!feeder_state_enabled) {
        draw_feeder(GRAPHIC_ADD);
        feeder_state_enabled = 1;
    } else{
        draw_feeder(GRAPHIC_MODIFY);
    }
}

/* ============================================================================
 * SIMPLE TEST GRAPHIC (Sent once on connect)
 * ============================================================================ */
//static void draw_test_shapes(uint8_t op)
//{
//    uint8_t  tx_buffer[128];
//    uint16_t pos;
//    graphic_data_struct_t *g;

//    // Clear the buffer to prevent random memory corruption
//    memset(tx_buffer, 0, sizeof(tx_buffer));

//    // We are drawing exactly 2 graphics, so we use count=2 (cmd_id 0x0102)
//    pos = build_graphic_header(tx_buffer, 2);
//    if (pos == 0) return;

//    /* Graphic 1: A prominent Pink Circle in the center */
//    g = (graphic_data_struct_t *)(tx_buffer + pos);
//    set_name(g, 'T', 'S', 'C');             // Name: TeSt Circle
//    g->layer = 8;                           // High layer to stay on top
//    g->color = GRAPHIC_COLOUR_PINK;         // Highly visible color
//    g->operation_type = op;
//    g->graphic_type = GRAPHIC_TYPE_CIRCLE;  
//    g->width = 5;                           // Line thickness
//    g->start_x = CENTER_X;                  // Center X
//    g->start_y = CENTER_Y;                  // Center Y
//    g->details_c = 150;                     // Radius of the circle
//    pos += sizeof(graphic_data_struct_t);

//    /* Graphic 2: A Yellow Rectangle around the circle */
//    g = (graphic_data_struct_t *)(tx_buffer + pos);
//    set_name(g, 'T', 'S', 'B');             // Name: TeSt Box
//    g->layer = 8;
//    g->color = GRAPHIC_COLOUR_YELLOW;
//    g->operation_type = op;
//    g->graphic_type = GRAPHIC_TYPE_RECTANGLE;
//    g->width = 4;
//    g->start_x = CENTER_X - 180;            // Bottom-Left X
//    g->start_y = CENTER_Y - 180;            // Bottom-Left Y
//    g->details_d = CENTER_X + 180;          // Top-Right X
//    g->details_e = CENTER_Y + 180;          // Top-Right Y
//    pos += sizeof(graphic_data_struct_t);

//    // Send the packet to the DMA
//    ref_send(tx_buffer, pos);
//}


/* ============================================================================
 * HIGH-LEVEL: ADD-ALL / MODIFY-ALL
 * ============================================================================ */
static void draw_all_static(uint8_t op)
{
    //draw_test_shapes(op);
    draw_crosshair(op);
    draw_pitch_ticks(op);
    draw_pitch_limits(op);
    draw_italy_flag_spqr(op);
}

static void draw_all_text(uint8_t op)
{
    draw_spin_text(op);
    draw_gear_text(op);
    draw_aim_text(op);
    draw_supercap_text(op);
}

static void update_text_if_changed(void)
{
    if (prev_spinspin != is_rotating) {
        prev_spinspin = is_rotating;
        draw_spin_text(GRAPHIC_MODIFY);
    }
    if (prev_gear != gear_speed_curr_gear) {
        prev_gear = gear_speed_curr_gear;
        draw_gear_text(GRAPHIC_MODIFY);
    }
    if (prev_aimbot != aimbot_mode) {
        prev_aimbot = aimbot_mode;
        draw_aim_text(GRAPHIC_MODIFY);
    }
    if (prev_supercap_dash != supercap_dash) {
        prev_supercap_dash = supercap_dash;
        draw_supercap_text(GRAPHIC_MODIFY);
    }
}

/* ============================================================================
 * STATE INGEST (read sensors -> UI globals)
 * ============================================================================ */
static void poll_inputs(void)
{
    uint16_t buf_e = Referee_System_Info.power_heat_data.buffer_energy;
    if (buf_e > 100) buf_e = 100;
    uint16_t mapped = (buf_e <= 60) ? (uint16_t)(buf_e * 1.66f) : buf_e;
    if (mapped > 100) mapped = 100;
    charging_state = (uint8_t)mapped;

    supercap_dash = RC_info.Key.Set.C ? 1 : 0;
    aimbot_mode   = RC_info.Mouse.Press_R ? 1 : 0;
}

/* ============================================================================
 * MAIN TASK
 * ============================================================================ */
void UI_Task(void const * argument)
{
    (void)argument;

    /* Period strictly set to match the 10Hz limit */
    const TickType_t xPeriod = 100 / portTICK_PERIOD_MS;

    /* Init synchronization primitives */
    osSemaphoreDef(UI_TX_DONE);
    ui_tx_done_sem = osSemaphoreCreate(osSemaphore(UI_TX_DONE), 1);
    osSemaphoreWait(ui_tx_done_sem, 0);

    osSemaphoreDef(UI_SEND_MTX);
    ui_send_mtx = osSemaphoreCreate(osSemaphore(UI_SEND_MTX), 1);

    set_top_coordinates();

    uint16_t current_robot_id = 0;
		uint8_t  prev_progress = 0;
		uint8_t  prev_q = 0;

    for (;;)
    {
        uint16_t rid = Referee_System_Info.robot_status.robot_id;
				uint8_t  progress = Referee_System_Info.game_status.game_progress;
				uint8_t  q = RC_info.Key.Set.Q;

				uint8_t id_changed   = (rid != 0 && rid != current_robot_id);
				uint8_t entered_game = (progress == 3 && prev_progress != 3);  // rising edge
				uint8_t q_pressed    = (q && !prev_q); 

        if (id_changed || entered_game || q_pressed) {
            current_robot_id = rid;
            map_robot_id(current_robot_id);
				
            osDelay(500);
            clear_hud();

            draw_all_static(GRAPHIC_ADD);
            draw_all_text(GRAPHIC_ADD);
            draw_dynamic(GRAPHIC_ADD);

            prev_spinspin      = -1;
            prev_aimbot        = -1;
            prev_supercap_dash = -1;
            prev_gear          = -1;
            prev_spin_warning  = -1;
            prev_motor_error   = -1;
            prev_feeder_state  = -1;
            motor_fault_enabled  = 0;
            feeder_state_enabled = 0;
        }
        else if (current_robot_id != 0) {
            poll_inputs();

            draw_dynamic(GRAPHIC_MODIFY);
            update_text_if_changed();
            update_spin_warning();
            update_motor_fault();
            update_feeder();
        }
				prev_progress = progress;
				prev_q = q;

        vTaskDelay(xPeriod);
    }
}