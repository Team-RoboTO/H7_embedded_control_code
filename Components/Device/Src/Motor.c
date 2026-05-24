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
#include "Motor.h"
#include "DJI_Motor.h"
#include "Damiao_Motor.h"
#include "Cubemars_Motor.h"
/* Ensure the header with the ID macros is included here, e.g.: */
// #include "Motor_Config.h" 

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
            .TxIdentifier = CM_CHASSIS_0_TX_ID,
            .RxIdentifier = CM_CHASSIS_0_RX_ID,
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
            .TxIdentifier = CM_CHASSIS_1_TX_ID,
            .RxIdentifier = CM_CHASSIS_1_RX_ID,
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
            .TxIdentifier = CM_CHASSIS_2_TX_ID,
            .RxIdentifier = CM_CHASSIS_2_RX_ID,
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
            .TxIdentifier = CM_CHASSIS_3_TX_ID,
            .RxIdentifier = CM_CHASSIS_3_RX_ID,
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
 * @brief Shooting wheel motors. 2 x DJI M3508.
 */
DJI_Motor_Info_Typedef DJI_Shooting_Motor[2] = {
    [0] = {
        .Type = DJI_M3508_SHOOTING_WHEELS,
        .FDCANFrame = {
            .TxIdentifier = DJI_SHOOTING_TX_ID,
            .RxIdentifier = DJI_SHOOTING_0_RX_ID,
        },
    },
    [1] = {
        .Type = DJI_M3508_SHOOTING_WHEELS,
        .FDCANFrame = {
            .TxIdentifier = DJI_SHOOTING_TX_ID,
            .RxIdentifier = DJI_SHOOTING_1_RX_ID,
        },
    },
};

//------------------------------------------------------------------------------
/**
 * @brief Lidar lifter motor. DJI M2006.
 */
DJI_Motor_Info_Typedef DJI_Lidar_Motor = {
		.Type = DJI_M2006,
		.FDCANFrame = {
				.TxIdentifier = DJI_LIDAR_TX_ID,
				.RxIdentifier = DJI_LIDAR_RX_ID,
		},
};

#if IS_SENTRY || IS_STD

	//------------------------------------------------------------------------------
	/**
	 * @brief Rev / feeder motor. DJI M2006.
	 */
	DJI_Motor_Info_Typedef DJI_Rev_Motor = {
			.Type = DJI_M2006,
			.FDCANFrame = {
					.TxIdentifier = DJI_REV_TX_ID,
					.RxIdentifier = DJI_REV_RX_ID,
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
					.P_MAX = 3.141592f,
					.V_MAX = 45.0f,
					.T_MAX = 12.0f,
			},
			.FDCANFrame = {
					.TxIdentifier = DM_YAW_TX_ID,
					.RxIdentifier = DM_YAW_RX_ID,
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
					.TxIdentifier = CM_PITCH_TX_ID,
					.RxIdentifier = CM_PITCH_RX_ID,
			},
			.Param_Range = {
					.P_MAX  = 3.141593f,
					.V_MAX  = 45.0f,
					.T_MAX  = 25.0f,
					.KP_MAX = 500.0f,
					.KD_MAX = 5.0f,
			},
	};

#elif IS_HERO
	
	//------------------------------------------------------------------------------
	/**
	 * @brief Push motor. DJI M2006.
	 */
	DJI_Motor_Info_Typedef DJI_Push_Motor = {
			.Type = DJI_M2006,
			.FDCANFrame = {
					.TxIdentifier = DJI_PUSH_TX_ID,
					.RxIdentifier = DJI_PUSH_RX_ID,
			},
	};
	
		//------------------------------------------------------------------------------
	/**
	 * @brief Yaw motor. DaMiao DM-J4310-2EC, MIT mode.
	 */
	DM_Motor_Info_Typedef DM_Yaw_Motor = {
			.Type = DM_MOTOR_J4310,
			.Control_Mode = MIT,
			.Param_Range = {
					.P_MAX = 3.141592f,
					.V_MAX = 45.0f,
					.T_MAX = 12.0f,
			},
			.FDCANFrame = {
					.TxIdentifier = DM_YAW_TX_ID,
					.RxIdentifier = DM_YAW_RX_ID,
			},
	};
	
		//------------------------------------------------------------------------------
	/**
	 * @brief Pitch motor. CubeMars AK60-06, MIT mode.
	 */
	CM_Motor_Info_Typedef CM_Pitch_Motor = {
			.Type = CM_AK60_06,
			.Control_Mode = CM_MIT_MODE,
			.FDCANFrame = {
					.TxIdentifier = CM_PITCH_TX_ID,
					.RxIdentifier = CM_PITCH_RX_ID,
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
	* @brief Yaw motor. DaMiao DM-J6006-2EC, MIT mode.
	*/
	DM_Motor_Info_Typedef DM_Rev_Motor = {
			.Type = DM_MOTOR_J6006,
			.Control_Mode = MIT,
			.Param_Range = {
					.P_MAX = 100.0f,
					.V_MAX = 45.0f,
					.T_MAX = 12.0f,
			},
			.FDCANFrame = {
					.TxIdentifier = DM_REV_TX_ID,
					.RxIdentifier = DM_REV_RX_ID,
			},
	};
	
#endif
