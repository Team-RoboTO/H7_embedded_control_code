/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Referee_System.h
  * @brief          : The header file of Referee_System.c
  * @author         : GrassFan Wang (Updated for RMUC 2026 V1.3.1)
  * @date           : 2026/05/21
  * @version        : v1.3.2
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef REFEREE_INFO_H
#define REFEREE_INFO_H

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "stdbool.h"

/* Exported defines ----------------------------------------------------------*/

#define REFEREE_RXFRAME_LENGTH  136   // frame_header 5bytes , cmd_id 2bytes , data_max 127bytes , crc16 2bytes = 136bytes 

/**
 * @brief Referee Communication protocol format
 */
#define FrameHeader_Length    5U   /*!< the length of frame header */
#define CMDID_Length          2U   /*!< the length of CMD ID */
#define CRC16_Length          2U   /*!< the length of CRC ID */

/**
 * @brief Cmd id
 */
#define GAME_STATUS_ID             0x0001U  /* game status data */
#define GAME_RESULT_ID             0x0002U  /* game result data */
#define GAME_ROBOT_HP_ID           0x0003U  /* robot HP data */
#define EVENET_DATA_ID             0x0101U  /* site event data */
#define REFEREE_WARNING_ID         0x0104U  /* referee warning data */
#define DART_INFO_ID               0x0105U  /* dart shoot data */
#define ROBOT_STATUS_ID            0x0201U  /* robot status data */
#define POWER_HEAT_DATA_ID         0x0202U  /* real power heat data */
#define ROBOT_POS_ID               0x0203U  /* robot position data */
#define BUFF_ID                    0x0204U  /* robot buff data */
#define HURT_DATA_ID               0x0206U  /* robot hurt data */
#define SHOOT_DATA_ID              0x0207U  /* real robot shoot data */
#define PROJECTILE_ALLOWANCE_ID    0x0208U  /* bullet remain data */
#define RFID_STATUS_ID             0x0209U  /* RFID status data */
#define DART_CLIENT_CMD_ID         0x020AU  /* DART Client cmd data */
#define GROUND_ROBOT_POSITION_ID   0x020BU  /* ground robot position */
#define RADAR_MARAKING_DATA_ID     0x020CU  /* Radar marking progress*/
#define SENTRY_INFO_ID             0x020DU  /* SENTRY make autonomous decisions*/
#define RADAR_INFO_ID              0x020EU  /* RADAR make autonomous decisions*/
#define ROBOT_INTERACTION_DATA_ID  0x0301U  /* robot interactive data */
#define MAP_COMMAND_ID             0x0303U  /* mini map interactive data */
#define MAP_ROBOT_DATA_ID          0x0305U  /* mini map receive data */
#define MAP_DATA_ID                0x0307U  /* mini map Auto Robot path */
#define CUSTOM_INFO_ID             0x0308U  /* mini map robot  path */

/* cancel byte alignment */
#pragma  pack(1)

/**
 * @brief typedef structure that contains the information of frame header
 */
typedef struct 
{
  uint8_t  SOF;           /*!< Data frame start byte, fixed value is 0xA5 */
  uint16_t Data_Length;   /*!< the length of data in the data frame */
  uint8_t  Seq;           /*!< package serial number */
  uint8_t  CRC8;          /*!< Frame header CRC8 checksum */
}FrameHeader_TypeDef;


/**
 * @brief typedef structure that contains the information of game status, id: 0x0001U
 */
typedef struct          
{
   uint8_t game_type : 4;        
   uint8_t game_progress : 4;       /*!< the progress of game */
   uint16_t stage_remain_time;      /*!< remain time of real progress */
   uint64_t SyncTimeStamp;         /*!< unix time */
       
}game_status_t;

/**
 * @brief typedef structure that contains the information of game result, id: 0x0002U
 */
typedef struct
{
 uint8_t winner;
}game_result_t;

/**
 * @brief typedef structure that contains the information of robot HP data, id: 0x0003U
 */
typedef struct
{
  uint16_t red_1_robot_HP;   
  uint16_t red_2_robot_HP;   
  uint16_t red_3_robot_HP;   
  uint16_t red_4_robot_HP;   
  uint16_t red_reserved;         
  uint16_t red_7_robot_HP;   
  uint16_t red_outpost_HP;   
  uint16_t red_base_HP;      

  uint16_t blue_1_robot_HP;   
  uint16_t blue_2_robot_HP;   
  uint16_t blue_3_robot_HP;   
  uint16_t blue_4_robot_HP;   
  uint16_t blue_reserved;  
  uint16_t blue_7_robot_HP;   
  uint16_t blue_outpost_HP;   
  uint16_t blue_base_HP;      
}game_robot_HP_t;

/**
 * @brief typedef structure that contains the information of site event data, id: 0x0101U
 *
 * V1.3.1 bit layout (RMUC 2026 protocol, Table 1-9):
 *  bit 0    : Own side Resupply Zone occupied (1 = occupied)
 *  bit 1    : Reserved
 *  bit 2    : Own side Resupply Zone occupied (RMUL only)
 *  bit 3-4  : Own side Small Power Rune status (0=inactive, 1=active, 2=activating)
 *  bit 5-6  : Own side Large Power Rune status (0=inactive, 1=active, 2=activating)
 *  bit 7-8  : Own side central elevated ground (0=none, 1=Own, 2=Opponent)
 *  bit 9-10 : Own side Trapezoid-Shaped Elevated Ground occupied
 *  bit 11-19: Time opponent's Dart last hit Own Outpost/Base (0-420 s, init 0)
 *  bit 20-22: Specific goal Opponent Dart hit
 *             (0=init, 1=Outpost, 2=Base fixed, 3=Base random fixed,
 *              4=Base random moving, 5=Base terminal moving)
 *  bit 23-24: Central Buff Point occupation (RMUL only; 0=none, 1=Own,
 *             2=Opponent, 3=both)
 *  bit 25-26: Own side Fortress Buff Point occupation
 *             (0=none, 1=Own, 2=Opponent, 3=both)
 *  bit 27-28: Own side Outpost Buff Point occupation (0=none, 1=Own, 2=Opponent)
 *  bit 29   : Own side Base Buff Point occupation (1=occupied)
 *  bit 30-31: Reserved
 */
typedef union
{
    uint32_t  event_data;
}event_data_t;

/**
 * @brief typedef structure that contains the warning  of Referee , id: 0x0104U
 */
typedef struct 
{ 
  uint8_t level; 
  uint8_t offending_robot_id; 
  uint8_t count; 
}referee_warning_t; 

/**
 * @brief typedef structure that contains the information of dart, id: 0x0105U
 */
typedef  struct
{
 uint8_t dart_remaining_time; /* The remaining time for our side's dart launcher, in seconds.*/
 uint16_t dart_info;
}dart_info_t;

/**
 * @brief typedef structure that contains the information of robot status, id: 0x0201U
 */
typedef struct
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;

  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;

  uint8_t power_management_gimbal_output : 1;
  uint8_t power_management_chassis_output : 1;
  uint8_t power_management_shooter_output : 1;
} robot_status_t;

/**
 * @brief typedef structure that contains the information of power heat data, id: 0x0202U
 */
typedef struct
{
  uint16_t reserved_1;
  uint16_t reserved_2;
  float reserved_3;
  uint16_t buffer_energy;
  uint16_t shooter_17mm_barrel_heat;
  uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/**
 * @brief typedef structure that contains the information of robot position data, id: 0x0203U
 */
typedef struct
{
  float x;      /* position x coordinate, unit: m */
  float y;      /* position y coordinate, unit: m */
  float angle;  /* Position muzzle, unit: degrees */
} robot_pos_t;

/**
 * @brief typedef structure that contains the information of robot buff data, id: 0x0204U
 */
typedef struct
{
   uint8_t recovery_buff;       /* Robot healing gain */
   uint16_t cooling_buff;       /* Robot shooting heat cooling rate */
   uint8_t defense_buff;        /* Robot defense gain */
   uint8_t vulnerability_buff;  /* Robot negative defense gain */
   uint16_t attack_buff;        /* Robot attack gain */
   uint8_t remaining_energy;    /* Feedback on the remaining energy value */
}buff_t;

/**
 * @brief typedef structure that contains the information of robot hurt, id: 0x0206U
 */
typedef struct
{
 uint8_t armor_id : 4; /* hurt armor id */
 uint8_t HP_deduction_reason : 4;
}hurt_data_t;

/**
 * @brief typedef structure that contains the information of real shoot data, id: 0x0207U
 */
typedef  struct
{
  uint8_t projectile_type;  /* 1:17mm 2:42mm */
  uint8_t shooter_number;
  uint8_t launching_frequency;  /* Hz */
  float projectile_speed;   /* m/s */
  uint16_t shot_count;      /* cumulative 0x0207 packets (one per shot), for CV */
}shoot_data_t;

/**
 * @brief typedef structure that contains the information of bullet remaining number, id: 0x0208U
 */
typedef  struct
{
  uint16_t projectile_allowance_17mm; 
  uint16_t projectile_allowance_42mm;  
  uint16_t remaining_gold_coin; 
  uint16_t projectile_allowance_fortress;
}projectile_allowance_t;

/**
 * @brief typedef structure that contains the information of RFID status, id: 0x0209U
 */
typedef struct
{
 uint32_t rfid_status;
 uint8_t rfid_status_2;
}rfid_status_t;

/**
 * @brief typedef structure that contains the information of dart client data, id: 0x020AU
 */
typedef  struct
{
 uint8_t dart_launch_opening_status;
 uint8_t reserved;
 uint16_t target_change_time;
 uint16_t latest_launch_cmd_time;
}dart_client_cmd_t;

/**
 * @brief typedef structure that contains the information of robot position in mimi map, id: 0x020BU
 */
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float infantry_3_x;
  float infantry_3_y;
  float infantry_4_x;
  float infantry_4_y;
  float reserved_1;
  float reserved_2;
}ground_robot_position_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020C
 */
typedef struct
{
  uint16_t tracking_progress; 
}radar_mark_data_t;

/**
 * @brief typedef structure that contains the information of robot mark, id: 0x020D
 */
typedef  struct
{
  uint32_t sentry_info; 
  uint16_t sentry_info_2; 
} sentry_info_t;

/**
 * @brief typedef structure that contains the information of radar, id: 0x020E
 */
typedef  struct
{
  uint8_t radar_info;
}radar_info_t;

/**
 * @brief typedef structure that contains the information of custom controller interactive, id: 0x0301U
 */
typedef struct{ 
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112];
}robot_interaction_data_t;

/**
 * @brief typedef structure that contains the information of client transmit data, id: 0x0303U
 */
typedef struct
{
  float opponent_position_x;
  float opponent_position_y;
  uint8_t cmd_keyboard;
  uint8_t opponent_robot_id;   
  uint16_t source_id;
}map_command_t;

/**
 * @brief typedef structure that contains the information of client receive data, id: 0x0305U
 */
typedef struct
{
    uint16_t opponent_hero_position_x;
    uint16_t opponent_hero_position_y;
    uint16_t opponent_engineer_position_x;
    uint16_t opponent_engineer_position_y;
    uint16_t opponent_infantry_3_position_x;
    uint16_t opponent_infantry_3_position_y;
    uint16_t opponent_infantry_4_position_x;
    uint16_t opponent_infantry_4_position_y;
    uint16_t opponent_aerial_position_x;
    uint16_t opponent_aerial_position_y;
    uint16_t opponent_sentry_position_x;
    uint16_t opponent_sentry_position_y;
    uint16_t ally_hero_position_x;
    uint16_t ally_hero_position_y;
    uint16_t ally_engineer_position_x;
    uint16_t ally_engineer_position_y;
    uint16_t ally_infantry_3_position_x;
    uint16_t ally_infantry_3_position_y;
    uint16_t ally_infantry_4_position_x;
    uint16_t ally_infantry_4_position_y;
    uint16_t ally_aerial_position_x;
    uint16_t ally_aerial_position_y;
    uint16_t ally_sentry_position_x;
    uint16_t ally_sentry_position_y;
}map_robot_data_t;

/**
 * @brief typedef structure that contains the information of sentry path, id: 0x0307U
 */
typedef struct
{
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
  uint16_t sender_id;
}map_data_t;

/**
 * @brief typedef structure that contains the information of custom info, id: 0x0308U
 */
typedef  struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
}custom_info_t;

/**
 * @brief typedef structure that contains the information of Referee
 */
typedef struct 
{
  uint8_t Index;
  uint16_t DataLength;
  
#ifdef GAME_STATUS_ID
  game_status_t game_status;
#endif

#ifdef GAME_RESULT_ID
  game_result_t game_result;
#endif

#ifdef  GAME_ROBOT_HP_ID
    game_robot_HP_t game_robot_HP;
#endif  
    
#ifdef EVENET_DATA_ID
  event_data_t event_data;
#endif

#ifdef REFEREE_WARNING_ID
    referee_warning_t referee_warning;
#endif

#ifdef DART_INFO_ID
  dart_info_t dart_info;
#endif

#ifdef ROBOT_STATUS_ID
  robot_status_t robot_status;
#endif

#ifdef POWER_HEAT_DATA_ID
  power_heat_data_t power_heat_data;
#endif

#ifdef ROBOT_POS_ID
  robot_pos_t  robot_pos;
#endif

#ifdef BUFF_ID
  buff_t  buff;
#endif

#ifdef HURT_DATA_ID
  hurt_data_t  hurt_data;
#endif

#ifdef SHOOT_DATA_ID
  shoot_data_t  shoot_data;
#endif

#ifdef PROJECTILE_ALLOWANCE_ID
  projectile_allowance_t projectile_allowance;
#endif

#ifdef  RFID_STATUS_ID
    rfid_status_t rfid_status;
#endif

#ifdef DART_CLIENT_CMD_ID
    dart_client_cmd_t  dart_client_cmd;
#endif

#ifdef GROUND_ROBOT_POSITION_ID
    ground_robot_position_t  ground_robot_position;
#endif 

#ifdef RADAR_MARAKING_DATA_ID
    radar_mark_data_t  radar_mark_data;
#endif

#ifdef SENTRY_INFO_ID
    sentry_info_t  sentry_info;
#endif

#ifdef RADAR_INFO_ID
    radar_info_t  radar_info;
#endif

#ifdef ROBOT_INTERACTION_DATA_ID
    robot_interaction_data_t robot_interaction_data;
#endif

#ifdef MAP_COMMAND_ID
    map_command_t map_command;
#endif

#ifdef MAP_ROBOT_DATA_ID
    map_robot_data_t  map_robot_data;
#endif

#ifdef MAP_DATA_ID
    map_data_t map_data;
#endif

#ifdef CUSTOM_INFO_ID
    custom_info_t custom_info;
#endif
}Referee_System_Info_TypeDef;

/* restore byte alignment */
#pragma  pack()

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief Referee_RxDMA MultiBuffer
 */
extern uint8_t Referee_System_Info_MultiRx_Buf[2][REFEREE_RXFRAME_LENGTH];
/**
 * @brief Referee structure variable
 */
extern Referee_System_Info_TypeDef Referee_System_Info;
/* Exported functions prototypes ---------------------------------------------*/
extern void Referee_System_Frame_Update(uint8_t *Buff);

#endif //REFEREE_INFO_H
