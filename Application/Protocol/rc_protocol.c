#include "rc_sensor.h"

#include "rc_protocol.h"

void Rc_Sensor_Init(rc_sensor_t *rc_sen)
{
	rc_sen->info->offline_cnt = rc_sen->info->offline_max_cnt + 1;
	rc_sen->work_state = Dvc_Offline;
	
	if(rc_sen->id == Dvc_Id_Rc)
		rc_sen->error = Dvc_None_Error;
	else
		rc_sen->error = Dvc_Id_Error;
}


short Key_W = 0, Key_A = 0, Key_S = 0, Key_D = 0;
void Rc_Sensor_Updata(rc_sensor_t *rc_sen, uint8_t *rxBuf)
{
	rc_sensor_info_t *rc_info = rc_sen->info;
	
	rc_info->ch0 = (rxBuf[0] | rxBuf[1] << 8) & 0x07FF;
	rc_info->ch0 -= 1024;
	rc_info->ch1 = (rxBuf[1] >> 3 | rxBuf[2] << 5) & 0x07FF;
	rc_info->ch1 -= 1024;
	rc_info->ch2 = (rxBuf[2] >> 6 | rxBuf[3] << 2 | rxBuf[4] << 10) & 0x07FF;
	rc_info->ch2 -= 1024;
	rc_info->ch3 = (rxBuf[4] >> 1 | rxBuf[5] << 7) & 0x07FF;
	rc_info->ch3 -= 1024;

	rc_info->s1 = ((rxBuf[5] >> 4) & 0x000C) >> 2;
	rc_info->s2 = (rxBuf[5] >> 4) & 0x0003;	
	
	rc_info->mouse_vx = rxBuf[6]  | (rxBuf[7] << 8);
	rc_info->mouse_vy = rxBuf[8]  | (rxBuf[9] << 8);
	rc_info->mouse_vz = rxBuf[10] | (rxBuf[11] << 8);
	rc_info->mouse_btn_l = rxBuf[12];
	rc_info->mouse_btn_r = rxBuf[13];
	
	rc_info->kb.key_v = rxBuf[14] | (rxBuf[15] << 8);	
	
	rc_info->thumbwheel = ((int16_t)rxBuf[16] | ((int16_t)rxBuf[17] << 8)) & 0x07ff;
	rc_info->thumbwheel -= 1024;
	
	rc_info->offline_cnt = 0;
		
}

/**
 *	@brief	在串口1中解析遥控数据协议
 */
void USART3_rxDataHandler(uint8_t *rxBuf)
{
	/*更新遥控器数据*/
	Rc_Sensor.update(&Rc_Sensor, rxBuf);
	Rc_Sensor.check(&Rc_Sensor);
}



