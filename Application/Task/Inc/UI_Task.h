#ifndef __UI_TASK_H
#define __UI_TASK_H

#include "usart.h"
#include "cmsis_os.h"


/* ============================================================================
 * REFEREE PROTOCOL CONSTANTS
 * ============================================================================ */
#define GRAPHIC_NO_OP       0
#define GRAPHIC_ADD         1
#define GRAPHIC_MODIFY      2
#define GRAPHIC_DELETE      3

#define GRAPHIC_TYPE_LINE       0
#define GRAPHIC_TYPE_RECTANGLE  1
#define GRAPHIC_TYPE_CIRCLE     2
#define GRAPHIC_TYPE_ELLIPSE    3
#define GRAPHIC_TYPE_ARC        4
#define GRAPHIC_TYPE_FLOAT      5
#define GRAPHIC_TYPE_INT        6
#define GRAPHIC_TYPE_CHAR       7

#define GRAPHIC_COLOUR_OWN_COLOR     0
#define GRAPHIC_COLOUR_YELLOW        1
#define GRAPHIC_COLOUR_GREEN         2
#define GRAPHIC_COLOUR_ORANGE        3
#define GRAPHIC_COLOUR_PURPLISH_RED  4
#define GRAPHIC_COLOUR_PINK          5
#define GRAPHIC_COLOUR_CYAN          6
#define GRAPHIC_COLOUR_BLACK         7
#define GRAPHIC_COLOUR_WHITE         8
#define GRAPHIC_COLOUR_RED		       9

/* ============================================================================
 * SCREEN LAYOUT
 * ============================================================================ */
#define HUD_MAX_X       1920
#define HUD_MAX_Y       1080
#define CENTER_X        (HUD_MAX_X / 2)
#if IS_STD || IS_SENTRY
#define CENTER_Y        ((HUD_MAX_Y / 2) - 35)
#else
#define CENTER_Y        (HUD_MAX_Y / 2)
#endif

/* ============================================================================
 * TEXT / GRAPHIC SIZES
 * ============================================================================ */
#define FONT_SIZE           20
#define FONT_SIZE_SMALL     15
#define CHAR_WIDTH          2
#define CHAR_WIDTH_SMALL    1
#define CHAR_X_OFFSET       12
#define CHAR_Y_OFFSET       12

/* ============================================================================
 * TOP BAR (SPIN / GEAR / AIM / CAP)
 * ============================================================================ */
#define TOP_Y_POS           900
#define TOP_GAP             150
#define BORDER_GAP_SIZE     15

/* ============================================================================
 * PITCH SCALE
 * ============================================================================ */
#define RADIAL_DIAMETER     200
#define ANGLE_LIMIT         60
#define MAJOR_TICK_LENGTH   20
#define MAJOR_TICK_WIDTH    3
#define MINOR_TICK_LENGTH   10
#define MINOR_TICK_WIDTH    2
#define TICK_COLOUR         GRAPHIC_COLOUR_WHITE
#define PITCH_LABEL_DIST    20
#define TICK_INTERVALS      15
#define PITCH_INVERT        1


/* Physical pitch range in degrees (±PITCH_RANGE_DEG).
 * Used to scale the arc indicator to cover the full ±ANGLE_LIMIT arc. */
#define PITCH_RANGE_DEG     30.0f


#define PITCH_BOUNDARY_COLOUR   GRAPHIC_COLOUR_PURPLISH_RED
#define PITCH_BOUNDARY_WIDTH    3
#define PITCH_ANG_COLOUR        GRAPHIC_COLOUR_GREEN
#define PITCH_ANG_WIDTH         5

/* ============================================================================
 * SUPERCAP / MOTOR FAULT
 * ============================================================================ */
#define SUPERCAP_ENABLE_THRESHOLD   50
#define SUPERCAP_DISABLE_THRESHOLD  10
#define MOTOR_FAULT_START           900
#define MOTOR_FAULT_GAP             50

/* ============================================================================
 * CROSSHAIR
 * ============================================================================ */
#define CROSSHAIR_COLOR     GRAPHIC_COLOUR_WHITE
#define SCALE_TICK_STEP     20

#ifndef PI
#define PI 3.1415926535f
#endif

/* ============================================================================
 * TIMING
 * ============================================================================ */
/* Referee system enforces ~10Hz per cmd_id. Keep send spacing >= 110ms when bursting. */
#define REF_DELAY           110

#pragma pack(1)

typedef struct __attribute__((packed)) {
    uint8_t  start_frame;
    uint16_t data_length;
    uint8_t  seq;
    uint8_t  crc;
    uint16_t cmd_id;
} ref_frame_header_t;

typedef struct __attribute__((packed)) {
    uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ref_inter_robot_data_t;

typedef struct __attribute__((packed)) {
    uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint8_t  graphic_operation;
    uint8_t  graphic_layer;
} ref_delete_graphic_t;

typedef struct __attribute__((packed)) {
    uint8_t  graphic_name[3];
    uint32_t operation_type:3;
    uint32_t graphic_type:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} graphic_data_struct_t;

#pragma pack()

typedef enum {
    FEEDER_STANDBY,
    FEEDER_SPINUP,
    FEEDER_LOADED,
    FEEDER_JAM,
    FEEDER_OVERHEAT,
    FEEDER_STEP,
    FEEDER_FIRING
} feeder_state_e;


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
#define MODE_LAYER    3

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */
void UI_Task(void const * argument);

/* Called from HAL_UART_TxCpltCallback for huart1 to signal DMA completion */
void UI_NotifyTxComplete(void);

#endif /* __UI_TASK_H */