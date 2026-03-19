/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : Motor.c
  * @brief          : Motor interfaces functions 
  * @author         : GrassFan Wang
  * @date           : 2025/01/22
  * @version        : v2.0
  ******************************************************************************
  * @attention      : None
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "DJI_Motor.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"

/* ===========================================================================
 *  LEGACY / REFERENCE DEFINITIONS (original robot config)
 * =========================================================================== */

/**
 * @brief Yaw motor (original). DJI GM6020.
 */
DJI_Motor_Info_Typedef DJI_Yaw_Motor =
{
    .Type = DJI_GM6020,
    .FDCANFrame = {
        .TxIdentifier = 0x1ff,
        .RxIdentifier = 0x205,
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Chassis motors (original). 4 x DJI M3508.
 */
DJI_Motor_Info_Typedef DJI_Chassis_Motor[4] = {
    [0] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x201,
        },
    },
    [1] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x202,
        },
    },
    [2] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x203,
        },
    },
    [3] = {
        .Type = DJI_M3508,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x204,
        },
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Joint motors (original). 4 x DaMiao DM-8009, MIT mode.
 */
DM_Motor_Info_Typedef DM_8009_Motor[4] = {
    [0] = {
        .Type = DM_MOTOR_8009,
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 3.141593f,
            .V_MAX = 45.0f,
            .T_MAX = 54.0f,
        },
        .FDCANFrame = {
            .TxIdentifier = 0x11,
            .RxIdentifier = 0x01,
        },
    },
    [1] = {
        .Type = DM_MOTOR_8009,
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 3.141593f,
            .V_MAX = 45.0f,
            .T_MAX = 54.0f,
        },
        .FDCANFrame = {
            .TxIdentifier = 0x12,
            .RxIdentifier = 0x02,
        },
    },
    [2] = {
        .Type = DM_MOTOR_8009,
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 3.141593f,
            .V_MAX = 45.0f,
            .T_MAX = 54.0f,
        },
        .FDCANFrame = {
            .TxIdentifier = 0x13,
            .RxIdentifier = 0x03,
        },
    },
    [3] = {
        .Type = DM_MOTOR_8009,
        .Control_Mode = MIT,
        .Param_Range = {
            .P_MAX = 3.141593f,
            .V_MAX = 45.0f,
            .T_MAX = 54.0f,
        },
        .FDCANFrame = {
            .TxIdentifier = 0x14,
            .RxIdentifier = 0x04,
        },
    },
};

DM_Motor_Control_Info_Typedef DM_Motor_Contorl_Info[4] = {0};

/* ===========================================================================
 *  NEW ROBOT DEFINITIONS
 *  Chassis:  4 x CubeMars AK40-10        (MIT mode)
 *  Yaw:      1 x DaMiao   DM-J6006-2EC   (MIT mode)
 *  Pitch:    1 x CubeMars AK40-10        (MIT mode)
 *  Shoot:    2 x DJI      M3508
 *  Rev:      1 x DJI      M2006
 * =========================================================================== */

//------------------------------------------------------------------------------
/**
 * @brief Chassis motors. 4 x CubeMars AK40-10, MIT mode.
 */
CM_Motor_Info_Typedef CM_Chassis_Motor[4] = {
    [0] = {
        .Type = CM_AK40_10,
        .Control_Mode = CM_MIT_MODE,
        .FDCANFrame = {
            .TxIdentifier = 123,
            .RxIdentifier = 0x0000007B,
        },
        .Param_Range = {
            .P_MAX  = 12.5f,
            .V_MAX  = 50.0f,
            .T_MAX  = 25.0f,
            .KP_MAX = 500.0f,
            .KD_MAX = 5.0f,
        },
    },
    [1] = {
        .Type = CM_AK40_10,
        .Control_Mode = CM_MIT_MODE,
        .FDCANFrame = {
            .TxIdentifier = 120,
            .RxIdentifier = 0x00000078,
        },
        .Param_Range = {
            .P_MAX  = 12.5f,
            .V_MAX  = 50.0f,
            .T_MAX  = 25.0f,
            .KP_MAX = 500.0f,
            .KD_MAX = 5.0f,
        },
    },
    [2] = {
        .Type = CM_AK40_10,
        .Control_Mode = CM_MIT_MODE,
        .FDCANFrame = {
            .TxIdentifier = 121,
            .RxIdentifier = 0x00000079,
        },
        .Param_Range = {
            .P_MAX  = 12.5f,
            .V_MAX  = 50.0f,
            .T_MAX  = 25.0f,
            .KP_MAX = 500.0f,
            .KD_MAX = 5.0f,
        },
    },
    [3] = {
        .Type = CM_AK40_10,
        .Control_Mode = CM_MIT_MODE,
        .FDCANFrame = {
            .TxIdentifier = 122,
            .RxIdentifier = 0x0000007A,
        },
        .Param_Range = {
            .P_MAX  = 12.5f,
            .V_MAX  = 50.0f,
            .T_MAX  = 25.0f,
            .KP_MAX = 500.0f,
            .KD_MAX = 5.0f,
        },
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Yaw motor. DaMiao DM-J6006-2EC, MIT mode.
 */
DM_Motor_Info_Typedef DM_Yaw_Motor = {
    .Type = DM_MOTOR_J6006,
    .Control_Mode = MIT,
    .Param_Range = {
        .P_MAX = 3.141593f,
        .V_MAX = 45.0f,
        .T_MAX = 12.0f,
    },
    .FDCANFrame = {
        .TxIdentifier = 0x11,
        .RxIdentifier = 0x01,
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Pitch motor. CubeMars AK40-10, MIT mode.
 */
CM_Motor_Info_Typedef CM_Pitch_Motor = {
    .Type = CM_AK40_10,
    .Control_Mode = CM_MIT_MODE,
    .FDCANFrame = {
        .TxIdentifier = 1,
        .RxIdentifier = 0x00000001,
    },
    .Param_Range = {
        .P_MAX  = 3.141593f,
        .V_MAX  = 45.0f,
        .T_MAX  = 25.0f,
        .KP_MAX = 500.0f,
        .KD_MAX = 5.0f,
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Shooting wheel motors. 2 x DJI M3508.
 */
DJI_Motor_Info_Typedef DJI_Shooting_Motor[2] = {
    [0] = {
        .Type = DJI_M3508_SHOOTING_WHEELS,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x201,
        },
    },
    [1] = {
        .Type = DJI_M3508_SHOOTING_WHEELS,
        .FDCANFrame = {
            .TxIdentifier = 0x200,
            .RxIdentifier = 0x202,
        },
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Rev / feeder motor. DJI M2006.
 */
DJI_Motor_Info_Typedef DJI_Rev_Motor = {
    .Type = DJI_M2006,
    .FDCANFrame = {
        .TxIdentifier = 0x200,
        .RxIdentifier = 0x203,
    },
};