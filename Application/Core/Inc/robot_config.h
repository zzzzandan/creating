#ifndef ROBOT_CONFIG__H__
#define ROBOT_CONFIG__H__

#include "stm32f4xx_hal.h"

#include <stdbool.h>


//#define PRINTF_DATA_ENABLE


/*----------�������----------*/
/*��������*/
typedef enum drv_type {
    DRV_CAN1,
    DRV_CAN2,
    DRV_IIC,
    DRV_UART1,
    DRV_SPI,
}drv_type_e;

/*SPI����*/
typedef struct drv_spi {
    drv_type_e 	type;
}drv_spi_t;

/*IIC����*/
typedef struct drv_iic {
    drv_type_e 	type;
}drv_iic_t;

/*CAN����*/
typedef struct drv_can {
    drv_type_e 	type;
    uint32_t				can_id;
    uint32_t				std_id;
    uint8_t					drv_id;
    void						(*tx_data)(struct drv_can *self, int16_t txData);
}drv_can_t;

/*UART����*/
typedef struct drv_uart {
    drv_type_e	type;
    void		(*tx_byte)(struct drv_uart *self, uint8_t byte);
}drv_uart_t;


/*�豸ID�б�*/
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


/*----------�������----------*/	
/*����ģʽ*/
typedef enum {
	Launcher_Mode_Normal,
	Launcher_Mode_Stop,
	Launcher_Mode_Debug,
}launcher_mode_e;

/*----------��̨���----------*/
/*��̨ģʽ*/
typedef enum {
	Gimbal_Mode_Normal,
	Gimbal_Mode_Paralyze,
	Gimbal_Mode_Stable,	
	Gimbal_Mode_Vision,
}gimbal_mode_e;
	
/*----------�������----------*/
/*����ģʽ*/
typedef enum {
	Chassis_Mode_Normal,
	Chassis_Mode_Paralyze,
	Chassis_Mode_Rotating,
}chassis_mode_e;

/*���������ǰ��*/
typedef enum {
	Logic_Front,
	Logic_Back
}logic_front_e;


/*�豸����״̬*/
typedef enum {
    Dvc_Online,
    Dvc_Offline,
}dvc_work_state_e;

/*�豸�������*/
typedef enum {
    Dvc_None_Error,		// ����(�޴���)
    Dvc_Id_Error,	// �豸ID����
    Dvc_Init_Error,	// �豸��ʼ������
    Dvc_Data_Error,	// �豸���ݴ���
}dvc_error_e;

/*Զ��ң��ģʽ*//*ң���� ����*/
typedef enum {
    RC,
    KEY,
}remote_mode_e;

/*ϵͳ״̬*/
typedef enum {
    Robot_State_Normal,
    Robot_State_RC_Lost,
    Robot_State_Gimbal_Motor_Last,
    Robot_State_Launcher_Motro_Last,
    Robot_State_Chassis_Motor_Last,
    Robot_State_Wrong,
}robot_state_e;

/*��־λ*/
typedef struct {
	struct {
		bool reset_start;
		bool reset_ok;
		bool gimbal_mode_lock;
		bool relife_flag;
	}gimbal;
}flag_t;

/*����������Ч��*/
typedef enum {
	Robot_Buff_Normal,
	Robot_Buff_Small,
	Robot_Buff_Big,
	Robot_Buff_Reload_Buff,
} robot_buff_e;

/*ϵͳģʽ*/
typedef struct {
	/*��̨ģ��ģʽ*/
	gimbal_mode_e	gimbal_mode;
	/*����ģ��ģʽ*/
	launcher_mode_e launcher_mode;
	/*����ģ��ģʽ*/
	chassis_mode_e	chassis_mode;
}robot_mode_t;

/*ϵͳ��Ϣ*/
typedef struct {
    /*ϵͳģʽ*/
    robot_mode_t*	robot_mode;
	/*���Ʒ�ʽ*/
    remote_mode_e	remote_mode;
    /*ϵͳ״̬*/
    robot_state_e	state;
    /*��Ϸbuff*/
	robot_buff_e	buff;
} robot_t;


#define Is_Key_Control 	Robot.remote_mode == KEY


extern flag_t	Flag;
extern robot_t  Robot;


#endif


