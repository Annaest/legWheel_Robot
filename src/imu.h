#ifndef __IMU_H__
#define __IMU_H__

#include "I2Cdev.h"
//IMU对象及数据
struct IMUData
{
	float yaw, pitch, roll;			 // rad
	float yawSpd, pitchSpd, rollSpd; // rad/s
	float zAccel,yAccel; // m/s^2
	float xgravity, ygravity, zgravity;


};
extern IMUData imuData;

void IMU_Task(void *arg);
void IMU_Init();
float flitter(float rato, float value, float &last_value);

#endif