#ifndef CHASSIS_MOTOR__H__
#define CHASSIS_MOTOR__H__

#include "robot_config.h"

/*���̵�������б�*/
typedef enum {
	Chassis_Motor_LF,
	Chassis_Motor_RF,
	Chassis_Motor_LB,
	Chassis_Motor_RB,
	Chassis_Motor_Count,
}chassis_motor_cnt_e;

/*���̵����Ϣ�ṹ��*/
typedef struct chassis_motor_info_struct {
	uint16_t	angle;/*��е�Ƕ�*/
	int16_t		speed;/*ת��*/
	int16_t		current;/*����*/

	uint16_t	angle_prev;/*ǰ��е�Ƕ�*/
	int32_t		angle_sum;/*���սǶ�*/

	uint8_t		init_flag;/*��ʼ����־λ*/
	uint8_t		offline_cnt;/*���߼���*/
	uint8_t		offline_max_cnt;/*������߼���ֵ*/	
} chassis_motor_info_t;

typedef struct chassis_motor_struct{
    chassis_motor_info_t	*info;		 
	drv_can_t				*driver;
    void					(*init)(struct chassis_motor_struct *self);	
    void					(*update)(struct chassis_motor_struct *self, uint8_t *rxBuf);	
    void					(*check)(struct chassis_motor_struct *self);	
    void					(*heart_beat)(struct chassis_motor_struct *self);	
    dvc_work_state_e		work_state;
    dvc_error_e				error;	
    dvc_id_t				id;	
} chassis_motor_t;

extern chassis_motor_info_t Chassis_Motor_Info[Chassis_Motor_Count];
extern chassis_motor_t Chassis_Motor[Chassis_Motor_Count];

#endif

