#include "config.h"
#include "motor.h"
#include "can.h"
#include "imu.h"
#include "control.h"

void Motor_SetVelocity(Motor* motor, float velocity);
void Motor_VelocityTask(void * arg);

Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;
// float pidWheelVel_Kp = 2.2;
// float pidWheelVel_Ki = 1.8;
// float pidWheelVel_Kd = 1.5;
float pidWheelVel_Kp = 0.4f;
float pidWheelVel_Ki = 0.08f;
float pidWheelVel_Kd = 0.2;
PidModle pidWheelVel_left = {pidWheelVel_Kp, pidWheelVel_Ki, pidWheelVel_Kd, 0,0,0,0,0};
PidModle pidWheelVel_right = {pidWheelVel_Kp, pidWheelVel_Ki, pidWheelVel_Kd,0,0,0,0,0};
/******* 电机模块 *******/

//初始化一个电机对象
void Motor_Init(Motor *motor, uint8_t id, float offsetAngle, float maxVoltage, float torqueRatio, float dir)
{
	motor->id = id;
	motor->speed = motor->angle = motor->target = 0;
	motor->offsetAngle = offsetAngle;
	motor->maxVoltage = maxVoltage;
	motor->torqueRatio = torqueRatio;
	motor->dir = dir;
	motor->speed_last = 0;
	
}


//初始化所有电机对象
//各个参数需要通过实际测量或拟合得到
void Motor_InitAll()
{
	Motor_Init(&leftJoint[0],  1, 5.31, 7, 0.0133f, -1);
	Motor_Init(&leftJoint[1],  2, 0.115, 7, 0.0133f, -1);
	Motor_Init(&leftWheel,     3, 0, 4.0f, 0.0096f, -1);
	Motor_Init(&rightJoint[0], 5, -2.72, 7, 0.0133f, -1);
	Motor_Init(&rightJoint[1], 6, 0.40, 7, 0.0133f, -1);
	Motor_Init(&rightWheel,    7, 0, 4.0f, 0.0101f, 1);
	xTaskCreate(Motor_SendTask, "Motor_SendTask", 2048, NULL, 1, NULL);
	xTaskCreate(Motor_VelocityTask, "Motor_VelocityTask", 2048, NULL, 3, NULL);
}

//从CAN总线接收到的数据中解析出电机角度和速度
void Motor_Update(Motor *motor, uint8_t *data)
{

	motor->angle = (*(int32_t *)&data[0] / 1000.0f - motor->offsetAngle) * motor->dir;
	motor->speed = (*(int16_t *)&data[4] / 10 * 2 * M_PI / 60) * motor->dir;
    motor->speed =  flitter(0.3, motor->speed, motor->speed_last);

	// Serial.print("速度：");
	// Serial.println(motor->speed);
}

//设置电机扭矩
void Motor_SetTorque(Motor *motor, float torque)
{
	motor->torque = torque;
}

void Motor_SetTargetVel(Motor *motor, float targetVel)
{
	motor->targetVel = targetVel;
}

//由设置的目标扭矩和当前转速计算补偿反电动势后的驱动输出电压，并进行限幅
//补偿的意义: 电机转速越快反电动势越大，需要加大驱动电压来抵消反电动势，使电流(扭矩)不随转速发生变化
void Motor_UpdateTarget(Motor *motor)
{
	if(motor->id == 3 || motor->id == 7){
		motor->target = motor->torque * motor->dir;
	}
	else{
		motor->target = motor->position;
	}

	
}

void Motor_SetPosition(Motor *motor, float position)
{
	motor->position = int((position*motor->dir + motor->offsetAngle)*100)/100.f;
}

float Motor_GetPosition(Motor *motor)
{
	return int(motor->angle*100)/100.f;
}


void Motor_SendTask(void *arg)
{
	uint8_t data[8] = {0};
	Motor* motorList[] = {&leftJoint[0], &leftJoint[1], &leftWheel, &rightJoint[0], &rightJoint[1], &rightWheel};
	while (1)
	{
		for (int i = 0; i < 6; i++){
			Motor_UpdateTarget(motorList[i]); 
		}	
		*(int16_t *)&data[0] = ((int16_t)(leftJoint[0].target * 1000));
		*(int16_t *)&data[2] = ((int16_t)(leftJoint[1].target * 1000));
		*(int16_t *)&data[4] = ((int16_t)(leftWheel.target * 1000));
		CAN_SendFrame(0x100, data);
		*(int16_t *)&data[0] = ((int16_t)(rightJoint[0].target * 1000));
		*(int16_t *)&data[2] = ((int16_t)(rightJoint[1].target * 1000));
		*(int16_t *)&data[4] = ((int16_t)(rightWheel.target * 1000));
		CAN_SendFrame(0x200, data);
		vTaskDelay(5);
	}
}

void Motor_VelocityTask(void * arg)
{
	while(1)
	{
		Motor_SetVelocity(&leftWheel, leftWheel.targetVel);
		Motor_SetVelocity(&rightWheel, rightWheel.targetVel);
		vTaskDelay(5);
	}
}



void Motor_SetVelocity(Motor* motor, float velocity)
{
	PidModle *pidWheelVel;
	float motor_UpMap = 0;
	float motor_DownMap = 0;
	float motor_trans = 0;
	if(motor->id == 3){
		pidWheelVel = &pidWheelVel_left;
		motor_UpMap = LEFT_MOTOR_UP_MAP;
		motor_DownMap = LEFT_MOTOR_DOWN_MAP;
	}
	else if(motor->id == 7){
		pidWheelVel = &pidWheelVel_right;
		motor_UpMap = RIGHT_MOTOR_UP_MAP;
		motor_DownMap = RIGHT_MOTOR_DOWN_MAP;
	}
	else {
		return;
	}


	pidWheelVel->error = velocity - motor->speed / motor_UpMap;

	pidWheelVel->pid_out += pidWheelVel->kp *(pidWheelVel->error - pidWheelVel->p_error) + 
								pidWheelVel->ki * pidWheelVel->error + 
								pidWheelVel->kd * ( pidWheelVel->p_error - 2 * pidWheelVel->p_error + pidWheelVel->pp_error);
	pidWheelVel->pp_error = pidWheelVel->p_error;   
	pidWheelVel->p_error = pidWheelVel->error;
	
	// float maxlimit = fabs(pidWheelVel->error)*10;
	// pidWheelVel->pid_out = pidWheelVel->pid_out > maxlimit ? maxlimit: pidWheelVel->pid_out;
	// pidWheelVel->pid_out = pidWheelVel->pid_out < -maxlimit ? -maxlimit: pidWheelVel->pid_out;

	// float maxlimit = fabs(pidWheelVel->error)*5;
	// pidWheelVel->pid_out = pidWheelVel->pid_out > maxlimit ? maxlimit: pidWheelVel->pid_out;
	// pidWheelVel->pid_out = pidWheelVel->pid_out < -maxlimit ? -maxlimit: pidWheelVel->pid_out;
	float targetV = (int)((velocity + pidWheelVel->pid_out)*100) / 100.0;
	targetV = targetV > 7 ? 7 : targetV;
	targetV = targetV < -7 ? -7 : targetV;
	
	// if(targetV > 0 && motor->id == 7){
	// 	targetV = targetV * RIGHT_MOTOR_UP_TRANS;
	// }
	// else if(targetV < 0 && motor->id == 7){
	// 	targetV = targetV * RIGHT_MOTOR_DOWN_TRANS;
	// }
	// else if(targetV > 0 && motor->id == 3){
	// 	targetV = targetV * LEFT_MOTOR_UP_TRANS;
	// }
	// else if(targetV < 0 && motor->id == 3){
	// 	targetV = targetV * LEFT_MOTOR_DOWN_TRANS;
	// }
	// if(motor->id == 7){
	// 	targetV *= MOTOR_SPEED_OFFSET;
	// }

	Motor_SetTorque(motor, targetV);

}