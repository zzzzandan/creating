#include "rc_sensor.h"

#include "rc_protocol.h"

/*取绝对值*/
#define abs(x) 					((x)>0? (x):(-(x)))

extern void Rc_Sensor_Init(rc_sensor_t *self);
extern void Rc_Sensor_Updata(rc_sensor_t *self, uint8_t *rxBuf);

static void Rc_Sensor_Check(rc_sensor_t *self);
static void Rc_Sensor_Heart_Beat(rc_sensor_t *self);

void RC_Reset_Data(rc_sensor_t *rc);


drv_uart_t	Rc_Sensor_Driver = {
	.type = DRV_UART1,
	.tx_byte = NULL,
};

// 遥控器信息
rc_sensor_info_t 	Rc_Sensor_Info = {
	.offline_max_cnt = 60,
};

rc_move_info_t Rc_Move_Info;

// 遥控器传感器
rc_sensor_t	Rc_Sensor = {
	.info = &Rc_Sensor_Info,								//数据结构体
	.move_info = &Rc_Move_Info,
	.init = Rc_Sensor_Init,									//传感器初始化
	.update = Rc_Sensor_Updata,							//数据更新
	.check = Rc_Sensor_Check,								//数据合理性判断
	.heart_beat = Rc_Sensor_Heart_Beat,			//状态更新
	.work_state = Dvc_Offline,							//状态查
	.id = Dvc_Id_Rc,
};


/*遥控器数据检查函数*/
static void Rc_Sensor_Check(rc_sensor_t *self)
{
	rc_sensor_info_t *rc_info = self->info;
	
	if(abs(rc_info->ch0) > 660 ||
	   abs(rc_info->ch1) > 660 ||
	   abs(rc_info->ch2) > 660 ||
	   abs(rc_info->ch3) > 660)
	{
		self->error = Dvc_Data_Error;
		rc_info->ch0 = 0;
		rc_info->ch1 = 0;
		rc_info->ch2 = 0;
		rc_info->ch3 = 0;		
		rc_info->s1 = RC_SW_MID;
		rc_info->s2 = RC_SW_MID;
		rc_info->thumbwheel = 0;
	}
	else
	{
		self->error = Dvc_None_Error;
	}
}

/*遥控器离线检测函数*/
static void Rc_Sensor_Heart_Beat(rc_sensor_t *self)
{
	rc_sensor_info_t *rc_info = self->info;

	/*离线记数器自增*/
	rc_info->offline_cnt++;

	/*离线记数大于设定值后切换状态为离线模式*/
	if(rc_info->offline_cnt > rc_info->offline_max_cnt)
	{
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		self->work_state = Dvc_Offline;
	} 
	
	/*进入Rc_Sensor_Updata函数后offline_cnt清零*/
	else
	{
		/* 离线->在线 */
		if(self->work_state == Dvc_Offline)
			self->work_state = Dvc_Online;
	}
}

/*遥控数据清空*//*失联时使用*/
void RC_Reset_Data(rc_sensor_t *rc)
{
	/*通道值强行设置成中间值(不拨动摇杆的状态)*/
	rc->info->ch0 = 0;
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;
	/*左右开关选择强行设置成中间值状态*/
	rc->info->s1 = RC_SW_MID;
	rc->info->s2 = RC_SW_MID;
	/*鼠标*/
	rc->info->mouse_vx = 0;
	rc->info->mouse_vy = 0;
	rc->info->mouse_vz = 0;
	rc->info->mouse_btn_l = 0;
	rc->info->mouse_btn_r = 0;
	/*键盘*/
	rc->info->kb.key_v = 0;
	/*左拨轮*/
	rc->info->thumbwheel = 0;
}


