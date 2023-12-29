#ifndef RC_SENSOR__H__
#define RC_SENSOR__H__

#include "robot_config.h"


/* ----------------------- RC Channel Definition------------------------------*/

#define    RC_CH_VALUE_MIN       ((uint16_t)364 )
#define    RC_CH_VALUE_OFFSET    ((uint16_t)1024)
#define	   RC_CH_VALUE_MAX       ((uint16_t)1684)
#define	   RC_CH_VALUE_SIDE_WIDTH	((RC_CH_VALUE_MAX-RC_CH_VALUE_MIN)/2)

/* ----------------------- RC Switch Definition-------------------------------*/

#define    RC_SW_UP              ((uint16_t)1)
#define    RC_SW_MID             ((uint16_t)3)
#define    RC_SW_DOWN            ((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */

#define    KEY_PRESSED_OFFSET_W        (Rc_Sensor_Info.kb.bit.W)
#define    KEY_PRESSED_OFFSET_S        (Rc_Sensor_Info.kb.bit.S)
#define    KEY_PRESSED_OFFSET_A        (Rc_Sensor_Info.kb.bit.A)
#define    KEY_PRESSED_OFFSET_D        (Rc_Sensor_Info.kb.bit.D)
#define    KEY_PRESSED_OFFSET_SHIFT    (Rc_Sensor_Info.kb.bit.SHIFT)
#define    KEY_PRESSED_OFFSET_CTRL     (Rc_Sensor_Info.kb.bit.CTRL)
#define    KEY_PRESSED_OFFSET_Q        (Rc_Sensor_Info.kb.bit.Q)
#define    KEY_PRESSED_OFFSET_E        (Rc_Sensor_Info.kb.bit.E)
#define    KEY_PRESSED_OFFSET_R        (Rc_Sensor_Info.kb.bit.R)
#define    KEY_PRESSED_OFFSET_F        (Rc_Sensor_Info.kb.bit.F)
#define    KEY_PRESSED_OFFSET_G        (Rc_Sensor_Info.kb.bit.G)
#define    KEY_PRESSED_OFFSET_Z        (Rc_Sensor_Info.kb.bit.Z)
#define    KEY_PRESSED_OFFSET_X        (Rc_Sensor_Info.kb.bit.X)
#define    KEY_PRESSED_OFFSET_C        (Rc_Sensor_Info.kb.bit.C)
#define    KEY_PRESSED_OFFSET_V        (Rc_Sensor_Info.kb.bit.V)
#define    KEY_PRESSED_OFFSET_B        (Rc_Sensor_Info.kb.bit.B)

/* ----------------------- Function Definition-------------------------------- */
/* 遥控摇杆通道偏移值 */
#define		RC_SW1_VALUE				(Rc_Sensor_Info.s1)
#define		RC_SW2_VALUE				(Rc_Sensor_Info.s2)
#define		RC_LEFT_CH_LR_VALUE			(Rc_Sensor_Info.ch2)
#define		RC_LEFT_CH_UD_VALUE			(Rc_Sensor_Info.ch3)
#define		RC_RIGH_CH_LR_VALUE			(Rc_Sensor_Info.ch0)
#define		RC_RIGH_CH_UD_VALUE			(Rc_Sensor_Info.ch1)
#define		RC_THUMB_WHEEL_VALUE		(Rc_Sensor_Info.thumbwheel)

/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (Rc_Sensor_Info.s1 == RC_SW_UP)
#define    IF_RC_SW1_MID     (Rc_Sensor_Info.s1 == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (Rc_Sensor_Info.s1 == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (Rc_Sensor_Info.s2 == RC_SW_UP)
#define    IF_RC_SW2_MID     (Rc_Sensor_Info.s2 == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (Rc_Sensor_Info.s2 == RC_SW_DOWN)

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (Rc_Sensor_Info.mouse_vx)
#define    MOUSE_Y_MOVE_SPEED    (Rc_Sensor_Info.mouse_vy)
#define    MOUSE_Z_MOVE_SPEED    (Rc_Sensor_Info.mouse_vz)


/* 检测鼠标按键状态 
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (Rc_Sensor_Info.mouse_btn_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (Rc_Sensor_Info.mouse_btn_r == 1)


/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  Rc_Sensor_Info.kb.key_v != 0  )
#define    IF_KEY_PRESSED_W       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (IF_KEY_PRESSED & KEY_PRESSED_OFFSET_SHIFT) != 0 )

typedef struct  rc_sensor_info_struct{
	int16_t 	ch0;
	int16_t 	ch1;
	int16_t 	ch2;
	int16_t 	ch3;
	uint8_t  	s1;
	uint8_t  	s2;
	int16_t		mouse_vx;
	int16_t 	mouse_vy;
	int16_t 	mouse_vz;
	uint8_t 	mouse_btn_l;
	uint8_t 	mouse_btn_r;
	
	union{
		uint16_t key_v;		
		struct
		{
			uint16_t W:1;
			uint16_t S:1;
			uint16_t A:1;
			uint16_t D:1;
			uint16_t SHIFT:1;
			uint16_t CTRL:1;
			uint16_t Q:1;
			uint16_t E:1;
			uint16_t R:1;
			uint16_t F:1;
			uint16_t G:1;
			uint16_t Z:1;
			uint16_t X:1;
			uint16_t C:1;
			uint16_t V:1;
			uint16_t B:1;
		}bit;
	}kb;
	int16_t 	thumbwheel;	
	
	int16_t		offline_cnt;
	int16_t		offline_max_cnt;
} rc_sensor_info_t;


typedef struct  rc_move_info_struct{
	float	mouse_vx;
	float 	mouse_vy;
	float 	mouse_vz;
	float   key_w;
	float   key_a;
	float   key_s;
	float   key_d;
} rc_move_info_t;


typedef struct  rc_sensor_struct{
	rc_sensor_info_t	*info;
	rc_move_info_t      *move_info;
	drv_uart_t			*driver;
	void				(*init)(struct rc_sensor_struct *self);
	void				(*update)(struct rc_sensor_struct *self, uint8_t *rxBuf);
	void				(*check)(struct rc_sensor_struct *self);	
	void				(*heart_beat)(struct rc_sensor_struct *self);
	dvc_work_state_e	work_state;
	dvc_error_e			error;
	dvc_id_t			id;
} rc_sensor_t;

extern rc_sensor_info_t Rc_Sensor_Info;
extern rc_sensor_t Rc_Sensor;
extern rc_move_info_t Rc_Move_Info ;
extern rc_sensor_t Rc_Sensor;

extern void RC_Reset_Data(rc_sensor_t *rc);


#endif

