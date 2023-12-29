#include "pid.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
#define constrain(x, min, max)	(((x)>(max))?(max):((x)<(min)?(min):(x)))



/*PID��ʼ��*//*����*/
void PID_Init(pid_ctrl_t *pid)
{
	pid->err = 0;
	pid->last_err = 0;
	pid->integral = 0;
	pid->pout = 0;
	pid->iout = 0;
	pid->dout = 0;
	pid->out = 0;
	pid->true_err = 0;
}

/*PID��������*/
void PID_Control_Single(pid_ctrl_t *pid)
{
	/*�������ֵ(��Ҫ���������м������)*/
	/*����*/
	pid->integral += pid->err;
	pid->integral = constrain(pid->integral, -(pid->integral_max)+pid->integral_bias, +(pid->integral_max)+pid->integral_bias);
	/*p i d ��������*/
	pid->pout = pid->kp * pid->err;
	pid->iout = pid->ki * pid->integral;
	pid->dout = pid->kd * (pid->err - pid->last_err);
	/*�ۼ�pid���ֵ*/
	pid->out = pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	/*��¼�ϴ����ֵ*/
	pid->last_err = pid->err;
}

void PID_Control_Single_Delta(pid_ctrl_t * pid)
{
	/*p i d ��������*/
	pid->pout = pid->kp * (pid->err - pid->last_err);
	pid->iout = pid->ki * pid->err;
	pid->dout = pid->kd * (pid->err - 2.0f * pid->last_err + pid->last_last_err);
	/*�ۼ�pid���ֵ*/
	pid->out += pid->pout + pid->iout + pid->dout;
	pid->out = constrain(pid->out, -pid->out_max, pid->out_max);
	/*��¼�ϴμ����ϴ����ֵ*/
	pid->last_err = pid->err;
	pid->last_last_err = pid->last_err;
}


