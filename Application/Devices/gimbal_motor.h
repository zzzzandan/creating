#ifndef GIMBAL_MOTOR__H__
#define GIMBAL_MOTOR__H__

#include "robot_config.h"

/*��̨תһ�ܶ�Ӧ�ĵ���Ƕ�*/
#define GIMBAL_YAW_CIRCULAR_STEP 8191

typedef enum {
	Gimbal_Yaw_Motor,
	Gimbal_Pitch_Motor,
	Gimbal_Motor_Count,
}gimbal_motor_cnt_e;

/*��̨���ģʽ*//*���ԽǶ�ģʽ ��ԽǶ�ģʽ*/
typedef enum {
	Absolute_Mode,
	Relative_Mode,
}gimbal_motor_mode_e;

/*��̨�����Ϣ�ṹ��*/
typedef struct gimbal_motor_info_struct {
	uint16_t	angle;/*��е�Ƕ�*/
	int16_t		speed;/*ת��*/
	int16_t		current;/*����*/
	uint16_t	angle_prev;/*ǰ��е�Ƕ�*/
	int32_t		angle_sum;/*���սǶ�*/
	uint8_t		init_flag;/*��ʼ����־λ*/
	uint8_t		offline_cnt;/*���߼���*/
	uint8_t		offline_max_cnt;/*������߼���ֵ*/	
} gimbal_motor_info_t;


typedef struct gimbal_motor_struct{
    gimbal_motor_info_t		*info;		 
		drv_can_t				*driver;
    void					(*init)(struct gimbal_motor_struct *self);	
    void					(*update)(struct gimbal_motor_struct *self, uint8_t *rxBuf);	
    void					(*check)(struct gimbal_motor_struct *self);	
    void					(*heart_beat)(struct gimbal_motor_struct *self);	
    dvc_work_state_e		work_state;
    dvc_error_e				error;	
    dvc_id_t				id;	
} gimbal_motor_t;

extern gimbal_motor_info_t Gimbal_Motor_Info[Gimbal_Motor_Count];
extern gimbal_motor_t Gimbal_Motor[Gimbal_Motor_Count];
extern gimbal_motor_info_t gimbal_yaw, gimbal_pitch;




#endif

