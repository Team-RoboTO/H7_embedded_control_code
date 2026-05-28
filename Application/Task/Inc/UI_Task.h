#ifndef __UI_TASK_H
#define __UI_TASK_H

#include "stdint.h"
#include "Referee_System.h"

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

/* ============================================================================
 * SCREEN LAYOUT
 * ============================================================================ */
#define HUD_MAX_X       1920
#define HUD_MAX_Y       1080
#define CENTER_X        (HUD_MAX_X / 2)
#define CENTER_Y        (HUD_MAX_Y / 2)

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
/* Delay interno a ref_send: basso perche' il rate e' controllato dalla state machine */
#define REF_DELAY           20

/* ============================================================================
 * INIT STATE MACHINE
 * ============================================================================ */
typedef enum {
    INIT_IDLE        = 0,
    INIT_CLEAR,           /*  1 - delete all */
    INIT_STATIC_1,        /*  2 - test shapes */
    INIT_STATIC_2,        /*  3 - crosshair pkt 1 (framework) */
    INIT_STATIC_3,        /*  4 - crosshair pkt 2 (tracks + corner) */
    INIT_STATIC_4,        /*  5 - crosshair pkt 3 (CBR + drop ticks) */
    INIT_TICKS_MAJ,       /*  6 - major pitch ticks */
    INIT_TICKS_MIN,       /*  7 - minor pitch ticks */
    INIT_TICKS_L0,        /*  8 - pitch label 0 */
    INIT_TICKS_L1,        /*  9 - pitch label 1 */
    INIT_TICKS_L2,        /* 10 - pitch label 2 */
    INIT_TICKS_L3,        /* 11 - pitch label 3 */
    INIT_TICKS_L4,        /* 12 - pitch label 4 */
    INIT_LIMITS,          /* 13 - pitch limits */
    INIT_TEXT_SPIN,       /* 14 - spin text */
    INIT_TEXT_GEAR,       /* 15 - gear text */
    INIT_TEXT_AIM,        /* 16 - aim text */
    INIT_TEXT_CAP,        /* 17 - supercap text */
    INIT_DYNAMIC,         /* 18 - dynamic graphics (arcs, bar, pitch line) */
    INIT_DONE,            /* 19 - init completo */
} init_state_e;

/* ============================================================================
 * PACKED STRUCTS
 * ============================================================================ */
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

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */
void UI_Task(void const * argument);

/* Called from HAL_UART_TxCpltCallback for huart1 to signal DMA completion */
void UI_NotifyTxComplete(void);

#endif /* __UI_TASK_H */