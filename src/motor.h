#ifndef __MOTOR_H__
#define __MOTOR_H__
#include <stdio.h>
#define MAP_VALUE                 18             // 11.6
#define MOTOR_SPEED_STANDER       40
#define MOTOR_SPEED_OFFSET        1.0
#define RIGHT_MOTOR_UP_MAP        MAP_VALUE   //6/60.5
#define RIGHT_MOTOR_DOWN_MAP      MAP_VALUE   //6/57.0
#define RIGHT_MOTOR_UP_TRANS      MOTOR_SPEED_STANDER/RIGHT_MOTOR_UP_MAP
#define RIGHT_MOTOR_DOWN_TRANS    MOTOR_SPEED_STANDER/RIGHT_MOTOR_DOWN_MAP

#define LEFT_MOTOR_UP_MAP         MAP_VALUE   //6/63.0
#define LEFT_MOTOR_DOWN_MAP       MAP_VALUE   //6/60.0
#define LEFT_MOTOR_UP_TRANS       MOTOR_SPEED_STANDER/LEFT_MOTOR_UP_MAP
#define LEFT_MOTOR_DOWN_TRANS     MOTOR_SPEED_STANDER/LEFT_MOTOR_DOWN_MAP

//����ṹ��
//leftJoint[0]:��ǰ�ؽڵ��, leftJoint[1]:���ؽڵ��, leftWheel:���ֵ��
//rightJoint[0]:��ǰ�ؽڵ��, rightJoint[1]:�Һ�ؽڵ��, rightWheel:�ҳ��ֵ��
typedef struct MotorModel Motor;
struct MotorModel
{
	uint8_t id;
	float speed;			   // rad/s
	float speed_last;
	float angle, offsetAngle;  // rad
	float target, maxVoltage; // V
	float targetVel;
	float torque, torqueRatio; // Nm, voltage = torque / torqueRatio
	float dir;				   // 1 or -1
	float position;            // rad
}; //�����������

void Motor_Init(Motor *motor, uint8_t id, float offsetAngle, float maxVoltage, float torqueRatio, float dir);
float Motor_CalcRevVolt4010(float speed);
float Motor_CalcRevVolt2804(float speed);
void Motor_InitAll();
void Motor_Update(Motor *motor, uint8_t *data);
void Motor_SetTorque(Motor *motor, float torque);
void Motor_Motor_UpdateTarget(Motor *motor);
void Motor_SendTask(void *arg);
void Motor_SetPosition(Motor *motor, float position);
float Motor_GetPosition(Motor *motor);
void Motor_SetTargetVel(Motor *motor, float targetVel);

extern Motor leftJoint[2], rightJoint[2], leftWheel, rightWheel;

#endif