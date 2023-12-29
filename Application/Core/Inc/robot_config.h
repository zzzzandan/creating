#ifndef ROBOT_CONFIG__H__
#define ROBOT_CONFIG__H__

#include "stm32f4xx_hal.h"

#include <stdbool.h>


//#define PRINTF_DATA_ENABLE


/*----------驱动相关----------*/
/*驱动类型*/
typedef enum drv_type {
    DRV_CAN1,
    DRV_CAN2,
    DRV_IIC,
    DRV_UART1,
    DRV_SPI,
}drv_type_e;

/*SPI驱动*/
typedef struct drv_spi {
    drv_type_e 	type;
}drv_spi_t;

/*IIC驱动*/
typedef struct drv_iic {
    drv_type_e 	type;
}drv_iic_t;

/*CAN驱动*/
typedef struct drv_can {
    drv_type_e 	type;
    uint32_t				can_id;
    uint32_t				std_id;
    uint8_t					drv_id;
    void						(*tx_data)(struct drv_can *self, int16_t txData);
}drv_can_t;

/*UART驱动*/
typedef struct drv_uart {
    drv_type_e	type;
    void		(*tx_byte)(struct drv_uart *self, uint8_t byte);
}drv_uart_t;


/*设备ID列表*/
typedef enum {
	Dvc_Id_Rc,
	Dvc_Id_Imu,
	Dvc_Id_Gimbal_Yaw,
	Dvc_Id_Gimbal_Pitch,
	Dvc_Id_Launcher_Frict_L,
	Dvc_Id_Launcher_Frict_R,
	Dvc_Id_Launcher_Dial,
	Dvc_Id_Launcher_Limit,
	Dvc_Chassis_Motor_LF,
	Dvc_Chassis_Motor_RF,
	Dvc_Chassis_Motor_LB,
	Dvc_Chassis_Motor_RB,
	DVC_ID_CNT,
}dvc_id_t;


/*----------发射相关----------*/	
/*发射模式*/
typedef enum {
	Launcher_Mode_Normal,
	Launcher_Mode_Stop,
	Launcher_Mode_Debug,
}launcher_mode_e;

/*----------云台相关----------*/
/*云台模式*/
typedef enum {
	Gimbal_Mode_Normal,
	Gimbal_Mode_Paralyze,
	Gimbal_Mode_Stable,	
	Gimbal_Mode_Vision,
}gimbal_mode_e;
	
/*----------底盘相关----------*/
/*底盘模式*/
typedef enum {
	Chassis_Mode_Normal,
	Chassis_Mode_Paralyze,
	Chassis_Mode_Rotating,
}chassis_mode_e;

/*定义底盘正前方*/
typedef enum {
	Logic_Front,
	Logic_Back
}logic_front_e;


/*设备工作状态*/
typedef enum {
    Dvc_Online,
    Dvc_Offline,
}dvc_work_state_e;

/*设备错误代码*/
typedef enum {
    Dvc_None_Error,		// 正常(无错误)
    Dvc_Id_Error,	// 设备ID错误
    Dvc_Init_Error,	// 设备初始化错误
    Dvc_Data_Error,	// 设备数据错误
}dvc_error_e;

/*远程遥控模式*//*遥控器 键盘*/
typedef enum {
    RC,
    KEY,
}remote_mode_e;

/*系统状态*/
typedef enum {
    Robot_State_Normal,
    Robot_State_RC_Lost,
    Robot_State_Gimbal_Motor_Last,
    Robot_State_Launcher_Motro_Last,
    Robot_State_Chassis_Motor_Last,
    Robot_State_Wrong,
}robot_state_e;

/*标志位*/
typedef struct {
	struct {
		bool reset_start;
		bool reset_ok;
		bool gimbal_mode_lock;
		bool relife_flag;
	}gimbal;
}flag_t;

/*机器人增益效果*/
typedef enum {
	Robot_Buff_Normal,
	Robot_Buff_Small,
	Robot_Buff_Big,
	Robot_Buff_Reload_Buff,
} robot_buff_e;

/*系统模式*/
typedef struct {
	/*云台模块模式*/
	gimbal_mode_e	gimbal_mode;
	/*发射模块模式*/
	launcher_mode_e launcher_mode;
	/*底盘模块模式*/
	chassis_mode_e	chassis_mode;
}robot_mode_t;

/*系统信息*/
typedef struct {
    /*系统模式*/
    robot_mode_t*	robot_mode;
	/*控制方式*/
    remote_mode_e	remote_mode;
    /*系统状态*/
    robot_state_e	state;
    /*游戏buff*/
	robot_buff_e	buff;
} robot_t;


#define Is_Key_Control 	Robot.remote_mode == KEY


extern flag_t	Flag;
extern robot_t  Robot;


#endif


