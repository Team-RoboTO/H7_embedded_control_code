/**
 * @file UI_Task.c
 * @brief Gestione dell'interfaccia grafica (HUD) per il pilota.
 * 
 * Descrizione degli elementi grafici visualizzati (Referee System):
 * --------------------------------------------------------------
 * 1. SPIN (Rotazione Chassis):
 *    - "ON" (Verde): Modalità "Spin" attiva (il robot ruota su se stesso).
 *    - "OFF" (Arancio): Chassis segue il gimbal normalmente.
 *    - Bordo: Un arco circolare che ruota insieme al telaio del robot.
 * 
 * 2. GEAR (Marcia/Velocità):
 *    - Indica il moltiplicatore di velocità attuale (es. GEAR 1, GEAR 2).
 * 
 * 3. AIM (Aimbot):
 *    - "AIM ON": Auto-aim attivato (segue il target tramite computer vision).
 *    - "AIM OFF": Mira manuale.
 * 
 * 4. CAP (Supercap):
 *    - Visualizza lo stato di carica del supercondensatore.
 *    - Arco radiale: Carica rimanente (Verde > soglia, Arancio < soglia).
 *    - "CAP ON/OFF": Stato di scarica del modulo di potenza.
 * 
 * 5. CROSSHAIR (Mirino):
 *    - Mirino stile Sniper con compensatore di caduta (tick verticali).
 *    - Cerchio centrale per la mira rapida.
 * 
 * 6. PITCH (Inclinazione):
 *    - Scala laterale che indica l'angolo di inclinazione della testa del robot.
 *    - Include limiti fisici e indicatore di posizione corrente.
 * 
 * 7. STATUS ALARMS:
 *    - "RUOTAAA!": Avviso visivo quando il robot è fermo (bersaglio facile).
 *    - MOTOR FAULT: Elenco motori in errore (es: FR, FL, BL, BR, YAW, PITCH).
 *    - FEEDER: Stato caricatore (JAMMED se bloccato, OVERHEAT se surriscaldato).
 */

#include "UI_Task.h"
#include "usart.h"
#include "cmsis_os.h"
#include "CRC.h"
#include <stdio.h>
#include <string.h>
#include "math.h"
#include "remote_control.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
#include "Chassis_control.h"
#include "INS_Task.h"

extern Referee_System_Info_TypeDef Referee_System_Info;
static uint16_t g_client_id = 0;
static uint8_t g_ref_tx_seq = 0;

// Enums and dummy variables to simulate the external states
enum feeder_state_e {
	FEEDER_STANDBY,
	FEEDER_SPINUP,
	FEEDER_LOADED,
	FEEDER_JAM,
	FEEDER_OVERHEAT,
	FEEDER_STEP,
	FEEDER_FIRING
};

enum feeder_state_e feeder_state = FEEDER_STANDBY;
int prev_feeder_state = 0;
int feeder_state_enabled = 0;


int prev_spinspin = 0;
int prev_spin_warning = 0;
int supercap_dash = 0;
int prev_supercap_dash = 0;
int aimbot_mode = 0;
int prev_aimbot = 0;
int gear_speed_curr_gear = 1;
int prev_gear = 0;
uint8_t charging_state = 50;

uint16_t g_motor_fault = 0;
int motor_fault_enabled = 0;
int prev_motor_error = 0;

static uint32_t spin_coords = 0;
static uint32_t gear_coords = 0;
static uint32_t aimbot_coords = 0;
static uint32_t supercap_coords = 0; // CORREZIONE: Aggiunto puntatore coords supercap

int top_graphics = 0;
int dynamic_graphics = 0;
float graphic_edge = 0; // Max/min angle in radians

// Using the user's remote control struct
extern NDJ6_Info_Typedef NDJ6_info;

// Using the user's motor structs
extern DM_Motor_Info_Typedef DM_Yaw_Motor;
extern CM_Motor_Info_Typedef CM_Pitch_Motor;
extern INS_Info_Typedef INS_Info; 

void map_robot_id(uint16_t robot_id){
	switch (robot_id) {
		case 1: g_client_id = 0x101; break;
		case 2: g_client_id = 0x102; break;
		case 3: g_client_id = 0x103; break;
		case 4: g_client_id = 0x104; break;
		case 5: g_client_id = 0x105; break;
		case 6: g_client_id = 0x106; break;
		case 7: g_client_id = 0x107; break;
		case 101: g_client_id = 0x165; break;
		case 102: g_client_id = 0x166; break;
		case 103: g_client_id = 0x167; break;
		case 104: g_client_id = 0x168; break;
		case 105: g_client_id = 0x169; break;
		case 106: g_client_id = 0x16A; break;
		case 107: g_client_id = 0x16B; break;
		default: g_client_id = 0; break;
	}
}

// Global counter to verify transmission in Watch window
volatile uint32_t ui_tx_count = 0;
// Mutex for safe UART transmission across multiple tasks
osSemaphoreId ui_send_sem;
// DMA buffer MUST be in AXI_SRAM and non-cacheable for H7
__attribute__((section (".AXI_SRAM"), aligned(32))) static uint8_t ui_tx_dma_buffer[1024];


void ref_send(uint8_t* tx_buffer, uint16_t tx_len){
	Append_CRC16_Check_Sum(tx_buffer, tx_len + 2);
	
	// If UART is in error state (e.g. overrun), reset it to READY to avoid hanging
	if (huart1.gState == HAL_UART_STATE_ERROR) {
		huart1.gState = HAL_UART_STATE_READY;
	}

	// Use semaphore to ensure only one task sends at a time
	if (ui_send_sem != NULL) {
		osSemaphoreWait(ui_send_sem, osWaitForever);
	}

	uint32_t wait_start = osKernelSysTick();
	while (huart1.gState != HAL_UART_STATE_READY) {
		osDelay(1);
		// 50ms timeout (100 bytes at 115200 baud takes ~8.6ms, but we need safe margin)
		if ((osKernelSysTick() - wait_start) > 50) {
			HAL_UART_AbortTransmit(&huart1);
			huart1.gState = HAL_UART_STATE_READY;
			break;
		}
	}
	
	// Copy to stable, DMA-accessible memory (AXI_SRAM)
	memcpy(ui_tx_dma_buffer, tx_buffer, tx_len + 2);
	
	// Clean D-Cache for the buffer to ensure DMA sees the fresh data on H7
	SCB_CleanDCache_by_Addr((uint32_t*)ui_tx_dma_buffer, tx_len + 2);
	
	// Increment counter for debugging (Watch window)
	ui_tx_count++;
	
	HAL_UART_Transmit_DMA(&huart1, ui_tx_dma_buffer, tx_len + 2);

	// Release semaphore after starting transmission
	// Note: In a perfect world, we'd release this in the TxCpltCallback, 
	// but since we check gState at the beginning, this is safe enough for now.
	if (ui_send_sem != NULL) {
		osSemaphoreRelease(ui_send_sem);
	}
}
void send_custom_ref_data(uint8_t* data, uint16_t len){
    if (len > 30) len = 30;
    uint8_t tx_buffer[256];
    ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = 6 + len;
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	
    uint16_t robot_id = Referee_System_Info.robot_status.robot_id;
    if (robot_id == 0) return;
    
    uint16_t client_id = 0;
    if (robot_id < 100) {
        client_id = 0x0100 + robot_id;
    } else {
        client_id = 0x0164 + (robot_id - 100);
    }
    
	ref_inter_robot_data_t* data_header = (ref_inter_robot_data_t*)(tx_buffer + 7);
	data_header->cmd_ID = 0x0200; 
	data_header->send_ID = robot_id;
	data_header->receiver_ID = client_id;
    
    memcpy(tx_buffer + 7 + sizeof(ref_inter_robot_data_t), data, len);
    uint16_t tx_len = 7 + sizeof(ref_inter_robot_data_t) + len;
    ref_send(tx_buffer, tx_len);
}

void set_top_coordinates(void) {
    // CORREZIONE: Array aumentato a 4 elementi e aggiunta l'impostazione per 4 elementi.
	uint32_t x_coordinates[4] = {0}; 
	top_graphics = 4; // SPINSPIN + GEARING + AIMBOT + SUPERCAP

	switch (top_graphics) {
		case 1: x_coordinates[0] = HUD_MAX_X / 2; break;
		case 2:
			x_coordinates[0] = HUD_MAX_X / 2 - TOP_GAP / 2;
			x_coordinates[1] = HUD_MAX_X / 2 + TOP_GAP / 2;
			break;
		case 3:
			x_coordinates[0] = HUD_MAX_X / 2 - TOP_GAP;
			x_coordinates[1] = HUD_MAX_X / 2;
			x_coordinates[2] = HUD_MAX_X / 2 + TOP_GAP;
			break;
        case 4:
		default:
            x_coordinates[0] = HUD_MAX_X / 2 - TOP_GAP * 1.5f;
            x_coordinates[1] = HUD_MAX_X / 2 - TOP_GAP / 2.0f;
            x_coordinates[2] = HUD_MAX_X / 2 + TOP_GAP / 2.0f;
            x_coordinates[3] = HUD_MAX_X / 2 + TOP_GAP * 1.5f;
			break;
	}

	int index = 0;
	spin_coords = x_coordinates[index++];
	gear_coords = x_coordinates[index++];
	aimbot_coords = x_coordinates[index++];
	supercap_coords = x_coordinates[index++]; // CORREZIONE: Assegnato correttamente senza sovrascrivere
	
    // CORREZIONE: Impostato a 5 poichè vengono disegnati effettivamente 5 elementi 
    // (spin(1) + supercap(1) + pitch(1) + balancing(2)).
	dynamic_graphics = 5; 
}

void clear_hud() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x0100;
	ref_delete->graphic_layer = 9;
	ref_delete->graphic_operation = 2; // delete all
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = Referee_System_Info.robot_status.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	osDelay(REF_DELAY);
}

uint16_t draw_empty(uint8_t* tx_buffer) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->operation_type = GRAPHIC_NO_OP;
	return sizeof(graphic_data_struct_t);
}

uint16_t draw_graphic_header(uint8_t* tx_buffer, uint8_t num_graphics){
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t) +
			sizeof(graphic_data_struct_t) * num_graphics;
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + 7);
	if (num_graphics == 1) graphic_header->cmd_ID = 0x0101;
	else if (num_graphics == 2) graphic_header->cmd_ID = 0x0102;
	else if (num_graphics <= 5) graphic_header->cmd_ID = 0x0103;
	else graphic_header->cmd_ID = 0x0104;
	
	graphic_header->send_ID = Referee_System_Info.robot_status.robot_id;
	graphic_header->receiver_ID = g_client_id;
	return sizeof(ref_frame_header_t)+sizeof(ref_inter_robot_data_t);
}

uint16_t draw_char_header(uint8_t* tx_buffer, uint8_t char_len){
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_inter_robot_data_t)
			+ sizeof(graphic_data_struct_t) + 30; // MUST be exactly 30 bytes per RM Protocol
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	ref_inter_robot_data_t* graphic_header = (ref_inter_robot_data_t*)(tx_buffer + sizeof(ref_frame_header_t));

	graphic_header->cmd_ID = 0x0110;
	graphic_header->send_ID = Referee_System_Info.robot_status.robot_id;
	graphic_header->receiver_ID = g_client_id;

	return sizeof(ref_frame_header_t)+sizeof(ref_inter_robot_data_t);
}

uint16_t draw_supercap(uint8_t* tx_buffer, uint8_t modify) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->color = (charging_state > SUPERCAP_ENABLE_THRESHOLD) ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
	graphic_data->graphic_name[0] = 'S';
	graphic_data->graphic_name[1] = 'U';
	graphic_data->graphic_name[2] = 'P';
	graphic_data->layer = 3;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_ARC;
	graphic_data->details_a = 270; 
    
	int supercap_range = 100 - SUPERCAP_DISABLE_THRESHOLD;
	float mapped_charging_state = fmaxf(0.0f, fminf(1.0f,
	    (float)(charging_state - SUPERCAP_DISABLE_THRESHOLD) / (float)supercap_range));

	int curr_lvl = fmaxf(1, (int)(mapped_charging_state * ANGLE_LIMIT));
	graphic_data->details_b = 270 + curr_lvl;

	graphic_data->width = 30; 
	graphic_data->start_x = HUD_MAX_X / 2; 
	graphic_data->start_y = HUD_MAX_Y / 2; 
	graphic_data->details_d = RADIAL_DIAMETER;
	graphic_data->details_e = RADIAL_DIAMETER;

	return sizeof(graphic_data_struct_t);
}

void draw_spin_char(uint8_t modify, uint32_t x_coords) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);
	char_len = is_rotating ?
			snprintf((char*) char_buffer, 30, "ON") :
			snprintf((char*) char_buffer, 30, "OFF");

	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);

	graphic_data->color = is_rotating ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
	graphic_data->graphic_name[0] = 'C';
	graphic_data->graphic_name[1] = 'H';
	graphic_data->graphic_name[2] = 'A';
	graphic_data->layer = 2;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;

	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
	graphic_data->details_a = FONT_SIZE; 
	graphic_data->details_b = char_len; 
	graphic_data->width = CHAR_WIDTH; 
	graphic_data->layer = 0;
	graphic_data->start_x = x_coords - CHAR_X_OFFSET * char_len;
	graphic_data->start_y = TOP_Y_POS + CHAR_Y_OFFSET;
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, 30);
	curr_pos += 30;

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void draw_spin_warning(uint8_t operation) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);
	uint8_t char_len = snprintf((char*) char_buffer, 30, "RUOTAAA!");

	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);

	graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;
	graphic_data->graphic_name[0] = 'W';
	graphic_data->graphic_name[1] = 'R';
	graphic_data->graphic_name[2] = 'N';
	graphic_data->layer = 9; // Livello alto per sovrapporsi a tutto
	graphic_data->operation_type = operation; // GRAPHIC_ADD, GRAPHIC_MODIFY, o GRAPHIC_DELETE

	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
	graphic_data->details_a = 60; 
	graphic_data->details_b = char_len; 
	graphic_data->width = 5; 
	graphic_data->start_x = CENTER_X - (25 * char_len); 
	graphic_data->start_y = CENTER_Y + 180;
	
	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, 30);
	curr_pos += 30;

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void check_spin_warning() {
    int current_warning = !is_rotating; // Sempre attivo se non gira

    // Se lo stato è cambiato (da off a on, o da on a off)
    if (current_warning != prev_spin_warning) {
        if (current_warning) {
            draw_spin_warning(GRAPHIC_ADD);
        } else {
            draw_spin_warning(GRAPHIC_DELETE);
        }
        prev_spin_warning = current_warning;
    }
}

uint16_t draw_spin_border(uint8_t* tx_buffer, uint8_t modify, uint32_t x_coords) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->color = is_rotating ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'O';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 2;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    
	float chassis_dir = DM_Yaw_Motor.Data.Angle * 57.2958f;
	graphic_data->graphic_type = GRAPHIC_TYPE_ARC;
	if (chassis_dir < 0) {
		graphic_data->details_a = 360 + chassis_dir + BORDER_GAP_SIZE;
		graphic_data->details_b = 360 + chassis_dir - BORDER_GAP_SIZE;
	} else {
		graphic_data->details_a = chassis_dir + BORDER_GAP_SIZE; 
		graphic_data->details_b = chassis_dir - BORDER_GAP_SIZE; 
	}

	graphic_data->width = 7; 
	graphic_data->start_x = x_coords;
	graphic_data->start_y = TOP_Y_POS;
	graphic_data->details_d = 50; 
	graphic_data->details_e = 50; 

	return sizeof(graphic_data_struct_t);
}

uint16_t draw_bullet_bar(uint8_t* tx_buffer, uint8_t modify){
	uint16_t curr_pos = 0;
    uint16_t bullets = Referee_System_Info.projectile_allowance.projectile_allowance_17mm;
    uint32_t max_bullets = 500;
    uint32_t max_h = 350; 
    uint32_t bar_x_base = 1890;

    if (bullets > max_bullets) bullets = max_bullets; 

    uint32_t bar_height = (bullets * max_h) / max_bullets;
    uint8_t color = GRAPHIC_COLOUR_GREEN;
    if (bullets < 50) color = GRAPHIC_COLOUR_PURPLISH_RED;
    else if (bullets < 150) color = GRAPHIC_COLOUR_YELLOW;

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'B';
	graphic_data->graphic_name[2] = 'O'; 
	graphic_data->layer = 3;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_RECTANGLE;
	graphic_data->color = GRAPHIC_COLOUR_WHITE;
	graphic_data->width = 3; // Thicker border
	graphic_data->start_x = bar_x_base - 15;
	graphic_data->start_y = 350;
	graphic_data->details_d = bar_x_base + 15;
	graphic_data->details_e = 350 + max_h;
	curr_pos += sizeof(graphic_data_struct_t);
    
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->graphic_name[0] = 'B';
	graphic_data->graphic_name[1] = 'B';
	graphic_data->graphic_name[2] = 'L'; 
	graphic_data->layer = 3;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_LINE; 
	graphic_data->color = color;
	graphic_data->width = 20; // Thicker bar
	graphic_data->start_x = bar_x_base;
	graphic_data->start_y = 351;
	graphic_data->details_d = bar_x_base;
	graphic_data->details_e = 351 + bar_height;
	curr_pos += sizeof(graphic_data_struct_t);
    

	return curr_pos;
}

void draw_aimbot(uint8_t modify, uint32_t x_coords) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);
	graphic_data_struct_t* graphic_data;
    
    // CORREZIONE: Ora mostra l'effettivo stato dell'aimbot, non quello del supercap
	char_len = aimbot_mode ?
			snprintf((char*) char_buffer, 30, "AIM ON") :
			snprintf((char*) char_buffer, 30, "AIM OFF");
	curr_pos = draw_char_header(tx_buffer, char_len);
	graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->color = aimbot_mode ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;

	graphic_data->graphic_name[0] = 'A';
	graphic_data->graphic_name[1] = 'I';
	graphic_data->graphic_name[2] = 'M';
	graphic_data->layer = 4;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; 
	graphic_data->details_a = FONT_SIZE; 
	graphic_data->details_b = char_len; 
	graphic_data->width = CHAR_WIDTH; 
	graphic_data->start_x = x_coords - CHAR_X_OFFSET * char_len;
	graphic_data->start_y = TOP_Y_POS + CHAR_Y_OFFSET;

	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, 30);
	curr_pos += 30;

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

// CORREZIONE: Nuova funzione aggiunta per splittare la logica supercap testo da aimbot testo
void draw_supercap_text(uint8_t modify, uint32_t x_coords) {
    uint8_t tx_buffer[256];
    uint8_t curr_pos = 0;
    uint8_t char_len = 0;
    char char_buffer[30];
    memset(char_buffer, 0, 30);
    graphic_data_struct_t* graphic_data;

    char_len = supercap_dash ?
            snprintf((char*) char_buffer, 30, "CAP ON") :
            snprintf((char*) char_buffer, 30, "CAP OFF");
    curr_pos = draw_char_header(tx_buffer, char_len);
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->color = supercap_dash ? GRAPHIC_COLOUR_GREEN : GRAPHIC_COLOUR_ORANGE;

    graphic_data->graphic_name[0] = 'C';
    graphic_data->graphic_name[1] = 'A';
    graphic_data->graphic_name[2] = 'P';
    graphic_data->layer = 4;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
    graphic_data->details_a = FONT_SIZE;
    graphic_data->details_b = char_len;
    graphic_data->width = CHAR_WIDTH;
    graphic_data->start_x = x_coords - CHAR_X_OFFSET * char_len;
    graphic_data->start_y = TOP_Y_POS + CHAR_Y_OFFSET;

    curr_pos += sizeof(graphic_data_struct_t);
    memcpy(tx_buffer + curr_pos, char_buffer, 30);
    curr_pos += 30;

    ref_send(tx_buffer, curr_pos);
    osDelay(REF_DELAY);
}

void draw_gearing(uint8_t modify, uint32_t x_coords) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);

	char_len = snprintf((char*) char_buffer, 30, "GEAR %d", gear_speed_curr_gear);
	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->color = GRAPHIC_COLOUR_CYAN;
	graphic_data->graphic_name[0] = 'G';
	graphic_data->graphic_name[1] = 'E';
	graphic_data->graphic_name[2] = 'A';
	graphic_data->layer = 5;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR; 
	graphic_data->details_a = FONT_SIZE; 
	graphic_data->details_b = char_len; 
	graphic_data->width = CHAR_WIDTH; 

	graphic_data->start_x = x_coords - CHAR_X_OFFSET * char_len;
	graphic_data->start_y = TOP_Y_POS + CHAR_Y_OFFSET;

	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, 30);
	curr_pos += 30;

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void draw_crosshair(uint8_t modify) {
    uint8_t tx_buffer[256];
    uint32_t curr_pos = 0;
    graphic_data_struct_t* graphic_data;

    // --- PACKET 1: Main Framework (5 Graphics) ---
    curr_pos = draw_graphic_header(tx_buffer, 5);
    
    // Left Wing Bracket (Thick)
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'W'; graphic_data->graphic_name[1] = 'L'; graphic_data->graphic_name[2] = 'B';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 5;
    graphic_data->start_x = CENTER_X - 180; graphic_data->start_y = CENTER_Y - 30;
    graphic_data->details_d = CENTER_X - 180; graphic_data->details_e = CENTER_Y + 30;
    curr_pos += sizeof(graphic_data_struct_t);

    // Right Wing Bracket (Thick)
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'W'; graphic_data->graphic_name[1] = 'R'; graphic_data->graphic_name[2] = 'B';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 5;
    graphic_data->start_x = CENTER_X + 180; graphic_data->start_y = CENTER_Y - 30;
    graphic_data->details_d = CENTER_X + 180; graphic_data->details_e = CENTER_Y + 30;
    curr_pos += sizeof(graphic_data_struct_t);

    // Main Horizontal Axis (Thick)
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'H'; graphic_data->graphic_name[1] = 'O'; graphic_data->graphic_name[2] = 'R';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = CROSSHAIR_COLOR;
    graphic_data->width = 4;
    graphic_data->start_x = CENTER_X - 160; graphic_data->start_y = CENTER_Y;
    graphic_data->details_d = CENTER_X + 160; graphic_data->details_e = CENTER_Y;
    curr_pos += sizeof(graphic_data_struct_t);

    // Vertical Top Mast
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'V'; graphic_data->graphic_name[1] = 'E'; graphic_data->graphic_name[2] = 'T';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = CROSSHAIR_COLOR;
    graphic_data->width = 4;
    graphic_data->start_x = CENTER_X; graphic_data->start_y = CENTER_Y + 40;
    graphic_data->details_d = CENTER_X; graphic_data->details_e = CENTER_Y + 140;
    curr_pos += sizeof(graphic_data_struct_t);

    // Center Diamond
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'D'; graphic_data->graphic_name[1] = 'I'; graphic_data->graphic_name[2] = 'A';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_RECTANGLE;
    graphic_data->color = GRAPHIC_COLOUR_ORANGE;
    graphic_data->width = 3;
    graphic_data->start_x = CENTER_X - 6; graphic_data->start_y = CENTER_Y - 6;
    graphic_data->details_d = CENTER_X + 6; graphic_data->details_e = CENTER_Y + 6;
    curr_pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, curr_pos);
    osDelay(REF_DELAY);

    // --- PACKET 2: Track Lines & Corner Brackets (7 Graphics) ---
    curr_pos = draw_graphic_header(tx_buffer, 7);
    
    // Left Track (Converging perspective)
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'T'; graphic_data->graphic_name[1] = 'R'; graphic_data->graphic_name[2] = 'L';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_GREEN;
    graphic_data->width = 3;
    graphic_data->start_x = CENTER_X - 40; graphic_data->start_y = CENTER_Y - 80; // Ends near center
    graphic_data->details_d = CENTER_X - 350; graphic_data->details_e = CENTER_Y - 540; // Starts wide at bottom
    curr_pos += sizeof(graphic_data_struct_t);

    // Right Track (Converging perspective)
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'T'; graphic_data->graphic_name[1] = 'R'; graphic_data->graphic_name[2] = 'R';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_GREEN;
    graphic_data->width = 3;
    graphic_data->start_x = CENTER_X + 40; graphic_data->start_y = CENTER_Y - 80; // Ends near center
    graphic_data->details_d = CENTER_X + 350; graphic_data->details_e = CENTER_Y - 540; // Starts wide at bottom
    curr_pos += sizeof(graphic_data_struct_t);

    // Bottom Perspective Bar
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'B'; graphic_data->graphic_name[1] = 'P'; graphic_data->graphic_name[2] = 'B';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_GREEN;
    graphic_data->width = 2;
    graphic_data->start_x = CENTER_X - 130; graphic_data->start_y = CENTER_Y - 500;
    graphic_data->details_d = CENTER_X + 130; graphic_data->details_e = CENTER_Y - 500;
    curr_pos += sizeof(graphic_data_struct_t);

    // Corner Brackets
    uint32_t cd = 110; uint32_t cl = 35;
    
    // CTL
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'C'; graphic_data->graphic_name[1] = 'T'; graphic_data->graphic_name[2] = 'L';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 2;
    graphic_data->start_x = CENTER_X - cd; graphic_data->start_y = CENTER_Y + cd;
    graphic_data->details_d = CENTER_X - cd + cl; graphic_data->details_e = CENTER_Y + cd - cl;
    curr_pos += sizeof(graphic_data_struct_t);

    // CTR
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'C'; graphic_data->graphic_name[1] = 'T'; graphic_data->graphic_name[2] = 'R';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 2;
    graphic_data->start_x = CENTER_X + cd; graphic_data->start_y = CENTER_Y + cd;
    graphic_data->details_d = CENTER_X + cd - cl; graphic_data->details_e = CENTER_Y + cd - cl;
    curr_pos += sizeof(graphic_data_struct_t);

    // CBL
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'C'; graphic_data->graphic_name[1] = 'B'; graphic_data->graphic_name[2] = 'L';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 2;
    graphic_data->start_x = CENTER_X - cd; graphic_data->start_y = CENTER_Y - cd;
    graphic_data->details_d = CENTER_X - cd + cl; graphic_data->details_e = CENTER_Y - cd + cl;
    curr_pos += sizeof(graphic_data_struct_t);

    // CBR
    graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
    graphic_data->graphic_name[0] = 'C'; graphic_data->graphic_name[1] = 'B'; graphic_data->graphic_name[2] = 'R';
    graphic_data->layer = 0;
    graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
    graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
    graphic_data->color = GRAPHIC_COLOUR_CYAN;
    graphic_data->width = 2;
    graphic_data->start_x = CENTER_X + cd; graphic_data->start_y = CENTER_Y - cd;
    graphic_data->details_d = CENTER_X + cd - cl; graphic_data->details_e = CENTER_Y - cd + cl;
    curr_pos += sizeof(graphic_data_struct_t);

    ref_send(tx_buffer, curr_pos);
    osDelay(REF_DELAY);

    // --- PACKET 3: Drop Compensator Ticks (7 Graphics) ---
    curr_pos = draw_graphic_header(tx_buffer, 7);
    for (int i = 0; i < 7; i++) {
        int y_offset = (i + 1) * SCALE_TICK_STEP;
        graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
        graphic_data->graphic_name[0] = 'T'; graphic_data->graphic_name[1] = 'K'; graphic_data->graphic_name[2] = 'A' + i;
        graphic_data->layer = 0;
        graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
        graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
        graphic_data->color = CROSSHAIR_COLOR;
        graphic_data->width = 1;
        int tick_w = 10 + (i * 2);
        graphic_data->start_x = CENTER_X - (tick_w / 2); graphic_data->start_y = CENTER_Y - y_offset; 
        graphic_data->details_d = CENTER_X + (tick_w / 2); graphic_data->details_e = CENTER_Y - y_offset;
        curr_pos += sizeof(graphic_data_struct_t);
    }
    ref_send(tx_buffer, curr_pos);
    osDelay(REF_DELAY);
}

void draw_major_ticks(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	curr_pos = draw_graphic_header(tx_buffer, 5);

	uint32_t xpos[5] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + RADIAL_DIAMETER,
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT * 0.0174533))
	};
	uint32_t ypos[5] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2,
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT * 0.0174533))
	};

	for (int i = 0; i < 5; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'M';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = MAJOR_TICK_WIDTH;
		graphic_data->start_x = xpos[i] - MAJOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MAJOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void draw_minor_ticks(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	curr_pos = draw_graphic_header(tx_buffer, 5);

	float gap = ANGLE_LIMIT / 4;
	uint32_t xpos[4] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(gap * 3 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(gap * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-gap * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-gap * 3 * 0.0174533))
	};
	uint32_t ypos[4] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(gap * 3 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(gap * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-gap * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-gap * 3 * 0.0174533))
	};

	for (int i = 0; i < 4; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'M';
		graphic_data->graphic_name[1] = 'I';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = MINOR_TICK_WIDTH;
		graphic_data->start_x = xpos[i] - MINOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MINOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}
	curr_pos += draw_empty(tx_buffer + curr_pos);

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void draw_pitch_labels(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;
	uint8_t char_len = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);
	int curr_tick_num = 2;

	uint32_t xpos[5] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + RADIAL_DIAMETER,
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(-ANGLE_LIMIT * 0.0174533))
	};
	uint32_t ypos[5] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2,
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT/2 * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(-ANGLE_LIMIT * 0.0174533))
	};

	for (int i = 0; i < 5; i++) {
		curr_pos = 0;
		char_len = snprintf((char*) char_buffer, 30, "%d", curr_tick_num * TICK_INTERVALS * 2);
		curr_tick_num--;
		curr_pos = draw_char_header(tx_buffer, char_len);

		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] =  i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
		graphic_data->color = TICK_COLOUR;
		graphic_data->width = CHAR_WIDTH_SMALL;
		graphic_data->start_x = xpos[i] + PITCH_LABEL_DIST;
		graphic_data->start_y = ypos[i] + CHAR_Y_OFFSET;
		graphic_data->details_a = FONT_SIZE_SMALL;
		graphic_data->details_b = char_len; 
		curr_pos += sizeof(graphic_data_struct_t);

		memcpy(tx_buffer + curr_pos, char_buffer, 30);
		curr_pos += 30;

		ref_send(tx_buffer, curr_pos);
		osDelay(REF_DELAY);
	}
}

void draw_pitch_limits(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint32_t curr_pos = 0;
	graphic_data_struct_t* graphic_data;

	float max_ang_pos = -PITCH_INVERT * INS_Info.Roll_Angle * ANGLE_LIMIT / graphic_edge;
	float min_ang_pos = -PITCH_INVERT * -INS_Info.Roll_Angle * ANGLE_LIMIT / graphic_edge;

	uint32_t xpos[2] = {
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(max_ang_pos * 0.0174533)),
			HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(min_ang_pos * 0.0174533))
	};
	uint32_t ypos[2] = {
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(max_ang_pos * 0.0174533)),
			HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(min_ang_pos * 0.0174533))
	};

	curr_pos = draw_graphic_header(tx_buffer, 2);

	for (int i = 0; i < 2; i++) {
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'I';
		graphic_data->graphic_name[2] = i + 1;
		graphic_data->layer = 1;
		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
		graphic_data->color = PITCH_BOUNDARY_COLOUR;
		graphic_data->width = PITCH_BOUNDARY_WIDTH;
		graphic_data->start_x = xpos[i] - MAJOR_TICK_LENGTH/2;
		graphic_data->start_y = ypos[i];
		graphic_data->details_d = xpos[i] + MAJOR_TICK_LENGTH/2;
		graphic_data->details_e = ypos[i];
		curr_pos += sizeof(graphic_data_struct_t);
	}

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

uint16_t draw_curr_pitch(uint8_t* tx_buffer, uint8_t modify) {
	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer);

    // Set scale to 1:6 (Angle / 6) to match the new HUD requirement
	float curr_ang_pos = -PITCH_INVERT * INS_Info.Roll_Angle * (ANGLE_LIMIT / 120.0f);
	uint32_t xpos = HUD_MAX_X/2 + (int)(RADIAL_DIAMETER*cos(curr_ang_pos * 0.0174533));
	uint32_t ypos = HUD_MAX_Y/2 + (int)(RADIAL_DIAMETER*sin(curr_ang_pos * 0.0174533));

	graphic_data->graphic_name[0] = 'P';
	graphic_data->graphic_name[1] = 'I';
	graphic_data->graphic_name[2] = 'T';
	graphic_data->layer = 1;
	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_LINE;
	graphic_data->color = PITCH_ANG_COLOUR;
	graphic_data->width = PITCH_ANG_WIDTH;
	graphic_data->start_x = xpos - MAJOR_TICK_LENGTH/2;
	graphic_data->start_y = ypos;
	graphic_data->details_d = xpos + MAJOR_TICK_LENGTH/2;
	graphic_data->details_e = ypos;

	return sizeof(graphic_data_struct_t);
}

void draw_pitch_graphics(uint8_t modify) {
	draw_major_ticks(modify);
	draw_minor_ticks(modify);
	draw_pitch_limits(modify);
	draw_pitch_labels(modify);
}

void motor_fault() {
	if (Referee_System_Info.game_status.game_progress == 4 || Referee_System_Info.game_status.game_progress == 3) {
		if (motor_fault_enabled) {
			motor_fault_enabled = 0;
			delete_motor_fault();
		}
	} else if (!motor_fault_enabled) {
		motor_fault_enabled = 1;
		draw_motor_fault(0);
		prev_motor_error = g_motor_fault;
	} else if (prev_motor_error != g_motor_fault) {
		delete_motor_fault();
		draw_motor_fault(0);
		prev_motor_error = g_motor_fault;
	}
}

void delete_motor_fault() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x0100;
	ref_delete->graphic_layer = 7;
	ref_delete->graphic_operation = 1; 
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = Referee_System_Info.robot_status.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	osDelay(REF_DELAY);
}

void draw_motor_fault(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30] = {0};
	uint8_t char_pos = 0;
	graphic_data_struct_t* graphic_data;

	for (uint8_t i = 0; i < 4; i++) {
		if (g_motor_fault & (1 << (i))) {
			if (strlen(char_buffer)) snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			switch (i) {
			case 0: snprintf(char_buffer + strlen(char_buffer), 30, "FR"); break;
			case 1: snprintf(char_buffer + strlen(char_buffer), 30, "FL"); break;
			case 2: snprintf(char_buffer + strlen(char_buffer), 30, "BL"); break;
			case 3: snprintf(char_buffer + strlen(char_buffer), 30, "BR"); break;
			}
		}
	}
	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		graphic_data->graphic_name[0] = 'C';
		graphic_data->graphic_name[1] = 'H';
		graphic_data->graphic_name[2] = 'S';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
		graphic_data->details_a = FONT_SIZE;
		graphic_data->details_b = char_len;
		graphic_data->width = CHAR_WIDTH;

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, 30);
		curr_pos += 30;

		ref_send(tx_buffer, curr_pos);
		osDelay(REF_DELAY);
		memset(char_buffer, 0, sizeof(char_buffer));
	}

	for (uint8_t i = 4; i < 7; i++) {
		if (g_motor_fault & (1 << (i))) {
			if (strlen(char_buffer)) snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			switch (i) {
			case 4: snprintf(char_buffer + strlen(char_buffer), 30, "LF"); break;
			case 5: snprintf(char_buffer + strlen(char_buffer), 30, "RF"); break;
			case 6: snprintf(char_buffer + strlen(char_buffer), 30, "FEED"); break;
			}
		}
	}

	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		graphic_data->graphic_name[0] = 'L';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = 'U';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
		graphic_data->details_a = FONT_SIZE;
		graphic_data->details_b = char_len;
		graphic_data->width = CHAR_WIDTH;

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, 30);
		curr_pos += 30;

		ref_send(tx_buffer, curr_pos);
		osDelay(REF_DELAY);
		memset(char_buffer, 0, sizeof(char_buffer));
	}

	for (uint8_t i = 7; i < 9; i++) {
		if (g_motor_fault & (1 << (i))) {
			if (strlen(char_buffer)) snprintf(char_buffer + strlen(char_buffer), 30, ", ");
			switch (i) {
			case 7: snprintf(char_buffer + strlen(char_buffer), 30, "PITCH"); break;
			case 8: snprintf(char_buffer + strlen(char_buffer), 30, "YAW"); break;
			}
		}
	}

	char_len = strlen(char_buffer);
	if (char_len) {
		curr_pos = draw_char_header(tx_buffer, char_len);
		graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
		graphic_data->color = GRAPHIC_COLOUR_PURPLISH_RED;

		graphic_data->graphic_name[0] = 'P';
		graphic_data->graphic_name[1] = 'A';
		graphic_data->graphic_name[2] = 'Y';
		graphic_data->layer = 7;

		graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
		graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
		graphic_data->details_a = FONT_SIZE;
		graphic_data->details_b = char_len;
		graphic_data->width = CHAR_WIDTH;

		graphic_data->start_x = 50;
		graphic_data->start_y = MOTOR_FAULT_START - MOTOR_FAULT_GAP * char_pos;
		char_pos++;

		curr_pos += sizeof(graphic_data_struct_t);
		memcpy(tx_buffer + curr_pos, char_buffer, 30);
		curr_pos += 30;

		ref_send(tx_buffer, curr_pos);
		osDelay(REF_DELAY);
	}
}

void draw_feeder_state(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	uint8_t char_len = 0;
	char char_buffer[30];
	memset(char_buffer, 0, 30);
	uint32_t colour = 0;

	switch (feeder_state) {
	case FEEDER_STANDBY:
	case FEEDER_SPINUP:
		colour = GRAPHIC_COLOUR_GREEN;
		char_len = snprintf((char*) char_buffer, 30, "STANDBY");
		break;
	case FEEDER_LOADED:
		colour = GRAPHIC_COLOUR_GREEN;
		char_len = snprintf((char*) char_buffer, 30, "LOADED");
		break;
	case FEEDER_JAM:
		colour = GRAPHIC_COLOUR_YELLOW;
		memset(char_buffer, 0, 30);
		char_len = snprintf((char*) char_buffer, 30, "JAMMED");
		break;
	case FEEDER_OVERHEAT:
		colour = GRAPHIC_COLOUR_PURPLISH_RED;
		memset(char_buffer, 0, 30);
		char_len = snprintf((char*) char_buffer, 30, "OVERHEAT");
		break;
	case FEEDER_STEP:
	case FEEDER_FIRING:
	default:
		colour = GRAPHIC_COLOUR_GREEN;
		memset(char_buffer, 0, 30);
		char_len = snprintf((char*) char_buffer, 30, "FIRING");
		break;
	}

	curr_pos = draw_char_header(tx_buffer, char_len);

	graphic_data_struct_t* graphic_data = (graphic_data_struct_t *)(tx_buffer + curr_pos);
	graphic_data->color = colour;

	graphic_data->graphic_name[0] = 'F';
	graphic_data->graphic_name[1] = 'D';
	graphic_data->graphic_name[2] = 'R';
	graphic_data->layer = 6;

	graphic_data->operation_type = modify ? GRAPHIC_MODIFY : GRAPHIC_ADD;
	graphic_data->graphic_type = GRAPHIC_TYPE_CHAR;
	graphic_data->details_a = FONT_SIZE; 
	graphic_data->details_b = char_len; 
	graphic_data->width = CHAR_WIDTH; 

	graphic_data->start_x = 50;
	graphic_data->start_y = 800;

	curr_pos += sizeof(graphic_data_struct_t);
	memcpy(tx_buffer + curr_pos, char_buffer, 30);
	curr_pos += 30;

	ref_send(tx_buffer, curr_pos);
	osDelay(REF_DELAY);
}

void delete_feeder_state() {
	uint8_t tx_buffer[256];
	ref_frame_header_t *send_header = (ref_frame_header_t*)tx_buffer;
	send_header->start_frame = 0xA5;
	send_header->seq = g_ref_tx_seq++;
	send_header->data_length = sizeof(ref_delete_graphic_t);
	Append_CRC8_Check_Sum(tx_buffer, 5);
	send_header->cmd_id = 0x0301;
	ref_delete_graphic_t *ref_delete = (ref_delete_graphic_t*)(tx_buffer + 7);
	ref_delete->cmd_ID = 0x0100;
	ref_delete->graphic_layer = 6;
	ref_delete->graphic_operation = 1; 
	ref_delete->receiver_ID = g_client_id;
	ref_delete->send_ID = Referee_System_Info.robot_status.robot_id;
	uint16_t tx_len = 7 + sizeof(ref_delete_graphic_t);
	ref_send(tx_buffer,tx_len);
	osDelay(REF_DELAY);
}

void dfeeder_state() {
	if (Referee_System_Info.game_status.game_progress == 4 || Referee_System_Info.game_status.game_progress == 3) {
		if (feeder_state_enabled) {
			feeder_state_enabled = 0;
			delete_feeder_state();
		}
	} else if (!feeder_state_enabled) {
		feeder_state_enabled = 1;
		draw_feeder_state(0);
		prev_feeder_state = feeder_state;
	} else if (prev_feeder_state != feeder_state) {
		draw_feeder_state(1);
		prev_feeder_state = feeder_state;
	}
}

void draw_static() {
	draw_crosshair(0);
	draw_pitch_graphics(0);
}

void draw_dynamic(uint8_t modify) {
	uint8_t tx_buffer[256];
	uint8_t curr_pos = 0;
	switch (dynamic_graphics) {
	case 0: break;
	case 1: curr_pos = draw_graphic_header(tx_buffer, 1); break;
	case 2: curr_pos = draw_graphic_header(tx_buffer, 2); break;
	case 3:
	case 4:
	case 5:
		curr_pos = draw_graphic_header(tx_buffer, 5);
		for (int i = 0; i < 5 - 5; i++) curr_pos += draw_empty(tx_buffer + curr_pos);
		break;
	case 6:
	case 7:
		curr_pos = draw_graphic_header(tx_buffer, 7);
		for (int i = 0; i < 7 - 5; i++) curr_pos += draw_empty(tx_buffer + curr_pos);
		break;
	}

    // Questi in totale generano 5 operazioni (1 + 1 + 1 + 2)
	curr_pos += draw_spin_border(tx_buffer + curr_pos, modify, spin_coords);
	curr_pos += draw_supercap(tx_buffer + curr_pos, modify);
	curr_pos += draw_curr_pitch(tx_buffer + curr_pos, modify);
	curr_pos += draw_bullet_bar(tx_buffer + curr_pos, modify);

	if (curr_pos) {
		ref_send(tx_buffer, curr_pos);
		osDelay(REF_DELAY);
	}
}

void draw_char(uint8_t modify) {
    static uint32_t refresh_counter = 0;
    bool force_update = (refresh_counter++ % 50 == 0); // Periodic refresh every ~5 seconds

	if (modify) {
		if (prev_spinspin != is_rotating || force_update) {
			prev_spinspin = is_rotating;
			draw_spin_char(modify, spin_coords);
		}
		if (prev_aimbot != aimbot_mode || force_update) {
			prev_aimbot = aimbot_mode;
			draw_aimbot(modify, aimbot_coords);
		}
		if (prev_supercap_dash != supercap_dash || force_update) {
			prev_supercap_dash = supercap_dash;
			draw_supercap_text(modify, supercap_coords);
		}
		if (prev_gear != gear_speed_curr_gear || force_update) {
			prev_gear = gear_speed_curr_gear;
			draw_gearing(modify, gear_coords);
		}
	} else {
		prev_spinspin = is_rotating;
		draw_spin_char(modify, spin_coords);
		prev_aimbot = aimbot_mode;
		draw_aimbot(modify, aimbot_coords);
		prev_supercap_dash = supercap_dash;
		draw_supercap_text(modify, supercap_coords);
		prev_gear = gear_speed_curr_gear;
		draw_gearing(modify, gear_coords);
	}
}

void UI_Task(void const * argument) {
	// Initialize the transmission semaphore
	osSemaphoreDef(UI_SEM);
	ui_send_sem = osSemaphoreCreate(osSemaphore(UI_SEM), 1);

	uint32_t start_wait = osKernelSysTick();
	while (Referee_System_Info.robot_status.robot_id == 0) {
		osDelay(10);
		if ((osKernelSysTick() - start_wait) > 500) {
			break;
		}
	}
	// Initial mapping
	map_robot_id(Referee_System_Info.robot_status.robot_id);


	set_top_coordinates();
	graphic_edge = TICK_INTERVALS * 4 * 0.0174533f;

	uint16_t current_robot_id = 0; // Inizializza a 0 per forzare il primo caricamento ADD nel loop

	// Inizializzazione spostata all'interno del while(1) per gestire meglio il rilevamento dell'ID
	// clear_hud();
	// draw_dynamic(0);
	// draw_char(0);
	// draw_static();

	while (1) {
		// If the real robot ID arrives and is different from what we initialized with,
		// we MUST re-send everything as ADD (0) because the new Client ID doesn't have the graphics yet.
		if (current_robot_id != Referee_System_Info.robot_status.robot_id && Referee_System_Info.robot_status.robot_id != 0) {
			current_robot_id = Referee_System_Info.robot_status.robot_id;
			map_robot_id(current_robot_id);
			
			osDelay(500); // Attendi che il client sia pronto
			clear_hud();
            
            // Send multiple ADD packets for redundancy (Referee Client is unreliable)
            for (int i = 0; i < 3; i++) {
			    draw_dynamic(0); // 0 = ADD
			    draw_char(0);    // 0 = ADD
			    draw_static();
                osDelay(200);
            }
            
			osDelay(1000); // Dai tempo di caricare tutto prima dei MODIFY
		} else if (current_robot_id != 0) {
			// Update mapping and states normally
			map_robot_id(current_robot_id);
            
            // Update UI states from sensors/referee
            charging_state = (uint8_t)(Referee_System_Info.power_heat_data.buffer_energy);
            if (charging_state <= 60) charging_state = (uint8_t)((float)charging_state * 1.66f);
            else if (charging_state > 100) charging_state = 100;
            
            supercap_dash = RC_info.Key.Set.C;
            aimbot_mode = (RC_info.Mouse.Press_R);
            // is_rotating is updated by state_machine toggle logic
		}
		
        if (current_robot_id != 0) {
		    draw_dynamic(1);
		    draw_char(1);
		    
		    motor_fault();
		    dfeeder_state();
		    check_spin_warning();
        }
		
        osDelay(100); 
	}
}