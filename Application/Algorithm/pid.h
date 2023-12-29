#ifndef PID_H
#define PID_H

#include "stdint.h"

/*PID控制结构体*/
typedef struct pid_ctrl {
	float	target;
	float	fdb;

	float 	err;
	float 	last_err;
	float	last_last_err;
	float 	true_err;

	float	kp;
	float 	ki;
	float 	kd;

	float 	pout;
	float 	iout;
	float 	dout;
	float 	out;

	float	integral;
	float 	integral_bias;

	float 	integral_max;
	float 	out_max;	
} pid_ctrl_t;


/*PID终端*/
typedef struct {  
	float  target;
	float  feedback;
	float  output;  
	
	/*微分滤波器系数 1、2*/
	float  c1;
	float  c2;
} pid_terminals_t;

/*PID参数*/
typedef struct {  
	float  Kr;    //reference set-point weighting 
	float  Kp;    //proportional loop gain
	float  Ki;    //integral gain
	float  Kd;    //derivative gain
	float  Km;    //derivative weighting
	
	float  Umax;  // Parameter: upper saturation limit
	float  Umin;  // Parameter: lower saturation limit
} pid_parameters_t;

/*PID数据*/
typedef struct {  
	float  up;    //proportional term
	float  ui;    //integral term
	float  ud;    //derivative term
	
	float  v1;    //pre-saturated controller output
	float  i1;    //integrator storage: ui(k-1)
	float  d1;    //differentiator storage: ud(k-1)
	float  d2;    //differentiator storage: d2(k-1) 
	float  w1;    //saturation record: [u(k-1) - v(k-1)]
} pid_data_t;
 
 
typedef struct {  
	pid_terminals_t term;
	pid_parameters_t param;
	pid_data_t  data;
} pid_controller_t;



extern void PID_Init(pid_ctrl_t *pid);
extern void PID_Control_Single(pid_ctrl_t *pid);
extern void PID_Control_Single_Delta(pid_ctrl_t *pid);

#endif
