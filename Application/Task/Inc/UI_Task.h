#ifndef __UI_TASK_H
#define __UI_TASK_H

#include "stdint.h"
#include "Referee_System.h"

// HUD Constants
#define GRAPHIC_NO_OP	0
#define GRAPHIC_ADD		1
#define GRAPHIC_MODIFY	2
#define GRAPHIC_DELETE	3

#define GRAPHIC_TYPE_LINE		0
#define GRAPHIC_TYPE_RECTANGLE	1
#define GRAPHIC_TYPE_CIRCLE		2
#define GRAPHIC_TYPE_ELLIPSE	3
#define GRAPHIC_TYPE_ARC		4
#define GRAPHIC_TYPE_FLOAT		5
#define GRAPHIC_TYPE_INT		6
#define GRAPHIC_TYPE_CHAR		7

#define GRAPHIC_COLOUR_OWN_COLOR		0
#define GRAPHIC_COLOUR_YELLOW			1
#define GRAPHIC_COLOUR_GREEN			2
#define GRAPHIC_COLOUR_ORANGE			3
#define GRAPHIC_COLOUR_PURPLISH_RED		4
#define GRAPHIC_COLOUR_PINK				5
#define GRAPHIC_COLOUR_CYAN				6
#define GRAPHIC_COLOUR_BLACK			7
#define GRAPHIC_COLOUR_WHITE			8

#define HUD_MAX_X 1920
#define HUD_MAX_Y 1080

// Graphic Sizes and Offsets
#define FONT_SIZE 20
#define FONT_SIZE_SMALL 15
#define CHAR_WIDTH 2
#define CHAR_WIDTH_SMALL 1
#define CHAR_X_OFFSET 12
#define CHAR_Y_OFFSET 12
#define BORDER_GAP_SIZE 15
#define TOP_Y_POS 900
#define TOP_GAP 150
#define RADIAL_DIAMETER 200
#define ANGLE_LIMIT 60
#define MAJOR_TICK_LENGTH 20
#define MAJOR_TICK_WIDTH 3
#define MINOR_TICK_LENGTH 10
#define MINOR_TICK_WIDTH 2
#define TICK_COLOUR GRAPHIC_COLOUR_WHITE
#define PITCH_LABEL_DIST 20
#define TICK_INTERVALS 15
#define PITCH_INVERT 1
#define YAW_INVERT 1
#define REMOTE_PITCH_SPEED 1.0f
#define REMOTE_YAW_SPEED 1.0f
#define MOTOR_FAULT_START 900
#define MOTOR_FAULT_GAP 50
#define SUPERCAP_ENABLE_THRESHOLD 50
#define SUPERCAP_DISABLE_THRESHOLD 10

#define PITCH_BOUNDARY_COLOUR GRAPHIC_COLOUR_PURPLISH_RED
#define PITCH_BOUNDARY_WIDTH 3
#define PITCH_ANG_COLOUR GRAPHIC_COLOUR_GREEN
#define PITCH_ANG_WIDTH 5

#define CROSSHAIR_COLOUR GRAPHIC_COLOUR_YELLOW
#define CROSSHAIR_THICKNESS 2
#define CROSSHAIR_CENTER_X (HUD_MAX_X/2)
#define CROSSHAIR_CENTER_Y (HUD_MAX_Y/2)
#define CROSSHAIR_START_X (CROSSHAIR_CENTER_X - 50)
#define CROSSHAIR_END_X (CROSSHAIR_CENTER_X + 50)
#define CROSSHAIR_START_Y (CROSSHAIR_CENTER_Y - 50)
#define CROSSHAIR_END_Y (CROSSHAIR_CENTER_Y + 50)

#define CROSSHAIR_DOT_COLOUR GRAPHIC_COLOUR_PINK
#define CROSSHAIR_DOT_WIDTH 4
#define CROSSHAIR_SHADOW_COLOUR GRAPHIC_COLOUR_BLACK
#define CROSSHAIR_SHADOW_THICKNESS 4

#define CROSSHAIR_TWO_COLOUR


// Definizioni per il nuovo mirino
#define CROSSHAIR_COLOR GRAPHIC_COLOUR_WHITE
#define SCALE_Y_START 200
#define SCALE_Y_END 800
#define SCALE_TICK_STEP 20
#define SCALE_TICK_WIDTH 30
#define CENTER_X 960  // Assumendo 1920/2
#define CENTER_Y 540  // Assumendo 1080/2
#ifndef PI
#define PI 3.1415926535f
#endif

#pragma pack(1)

typedef struct
{
	uint8_t start_frame;
	uint16_t data_length;
	uint8_t seq;
	uint8_t crc;
	uint16_t cmd_id;
} ref_frame_header_t;

typedef struct
{
	uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
} ref_inter_robot_data_t;

typedef struct
{
	uint16_t cmd_ID;
    uint16_t send_ID;
    uint16_t receiver_ID;
    uint8_t graphic_operation;
    uint8_t graphic_layer;
} ref_delete_graphic_t;

typedef struct
{
    uint8_t graphic_name[3];
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

#define REF_DELAY 100

// Function Prototypes
void UI_Task(void const * argument);
void map_robot_id(uint16_t robot_id);
void clear_hud(void);
void draw_static(void);
void draw_dynamic(uint8_t modify);
void draw_char(uint8_t modify);

uint16_t draw_graphic_header(uint8_t* tx_buffer, uint8_t num_graphics);
uint16_t draw_char_header(uint8_t* tx_buffer, uint8_t char_len);
void ref_send(uint8_t* tx_buffer, uint16_t tx_len);
void send_custom_ref_data(uint8_t* data, uint16_t len);

uint16_t draw_empty(uint8_t* tx_buffer);
void draw_crosshair(uint8_t modify);
void draw_aimbot(uint8_t modify, uint32_t x_coords);
void draw_supercap_text(uint8_t modify, uint32_t x_coords); // NUOVA FUNZIONE
void draw_gearing(uint8_t modify, uint32_t x_coords);
uint16_t draw_supercap(uint8_t* tx_buffer, uint8_t modify);

void draw_pitch_graphics(uint8_t modify);
void draw_major_ticks(uint8_t modify);
void draw_minor_ticks(uint8_t modify);
void draw_pitch_labels(uint8_t modify);
void draw_pitch_limits(uint8_t modify);
uint16_t draw_curr_pitch(uint8_t* tx_buffer, uint8_t modify);

void motor_fault(void);
void delete_motor_fault(void);
void draw_motor_fault(uint8_t modify);

void draw_feeder_state(uint8_t modify);
void delete_feeder_state(void);
void dfeeder_state(void);

uint16_t draw_balancing_status(uint8_t* tx_buffer, uint8_t modify);
void draw_spin_char(uint8_t modify, uint32_t x_coords);
uint16_t draw_spin_border(uint8_t* tx_buffer, uint8_t modify, uint32_t x_coords);

#endif /* __UI_TASK_H */