#include "rc_sensor.h"

#include "rc_protocol.h"

/*ȡ����ֵ*/
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

// ң������Ϣ
rc_sensor_info_t 	Rc_Sensor_Info = {
	.offline_max_cnt = 60,
};

rc_move_info_t Rc_Move_Info;

// ң����������
rc_sensor_t	Rc_Sensor = {
	.info = &Rc_Sensor_Info,								//���ݽṹ��
	.move_info = &Rc_Move_Info,
	.init = Rc_Sensor_Init,									//��������ʼ��
	.update = Rc_Sensor_Updata,							//���ݸ���
	.check = Rc_Sensor_Check,								//���ݺ������ж�
	.heart_beat = Rc_Sensor_Heart_Beat,			//״̬����
	.work_state = Dvc_Offline,							//״̬��
	.id = Dvc_Id_Rc,
};


/*ң�������ݼ�麯��*/
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

/*ң�������߼�⺯��*/
static void Rc_Sensor_Heart_Beat(rc_sensor_t *self)
{
	rc_sensor_info_t *rc_info = self->info;

	/*���߼���������*/
	rc_info->offline_cnt++;

	/*���߼��������趨ֵ���л�״̬Ϊ����ģʽ*/
	if(rc_info->offline_cnt > rc_info->offline_max_cnt)
	{
		rc_info->offline_cnt = rc_info->offline_max_cnt;
		self->work_state = Dvc_Offline;
	} 
	
	/*����Rc_Sensor_Updata������offline_cnt����*/
	else
	{
		/* ����->���� */
		if(self->work_state == Dvc_Offline)
			self->work_state = Dvc_Online;
	}
}

/*ң���������*//*ʧ��ʱʹ��*/
void RC_Reset_Data(rc_sensor_t *rc)
{
	/*ͨ��ֵǿ�����ó��м�ֵ(������ҡ�˵�״̬)*/
	rc->info->ch0 = 0;
	rc->info->ch1 = 0;
	rc->info->ch2 = 0;
	rc->info->ch3 = 0;
	/*���ҿ���ѡ��ǿ�����ó��м�ֵ״̬*/
	rc->info->s1 = RC_SW_MID;
	rc->info->s2 = RC_SW_MID;
	/*���*/
	rc->info->mouse_vx = 0;
	rc->info->mouse_vy = 0;
	rc->info->mouse_vz = 0;
	rc->info->mouse_btn_l = 0;
	rc->info->mouse_btn_r = 0;
	/*����*/
	rc->info->kb.key_v = 0;
	/*����*/
	rc->info->thumbwheel = 0;
}


