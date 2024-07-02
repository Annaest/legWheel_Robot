#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "config.h"

#define STATE_NORMAL   0
#define STATE_JUMP     1

#define JUMP_POSITON_LOW  1.5
#define JUMP_POSITON_HIGH 0.8
void Ctrl_Init();
void Ctrl_Task(void *arg);
void Operator_Task(void * args);
float nonlinear(float value);
float nolinar_error(float k,float error);
void Yaw_Ctrl(float in,float &out);

struct PidModle{
    float kp;
	float ki;
	float kd;
	float pid_out;
	float error;
	float p_error;
    float pp_error;
	float I;

};

extern float setLegPos; 
extern float speed[2];
extern float targetYaw;
extern float workState;
#endif