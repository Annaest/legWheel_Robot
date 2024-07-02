#include "imu.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "config.h"


VectorInt16 minusGravity(VectorInt16 vec);

MPU6050 mpu;
IMUData imuData;

/******* 陀螺仪模块 *******/

//陀螺仪数据获取任务
void IMU_Task(void *arg)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	uint8_t fifoBuffer[64]; //dmp数据接收区
	int16_t yawRound = 0; //统计yaw转过的整圈数
	float lastYaw = 0;
	float yAccelP = 0;
	float zAccelP = 0;
	float flitterLastYaw = 0;
	float yawOffset = 0;
	while (1)
	{
		if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
		{
			//获取陀螺仪角速度
			int16_t gyroData[3];
			mpu.getRotation(&gyroData[0], &gyroData[1], &gyroData[2]);
			imuData.rollSpd = gyroData[1] / 16.4f * M_PI / 180.0f;
			imuData.pitchSpd = -gyroData[0] / 16.4f * M_PI / 180.0f;
			imuData.yawSpd = gyroData[2] / 16.4f * M_PI / 180.0f;
			
			//获取陀螺仪欧拉角
			float ypr[3];
			Quaternion q;
			VectorFloat gravity;
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			float yaw = -ypr[0];
			imuData.pitch = -ypr[2];
			imuData.roll = -ypr[1];

			yawOffset += 0.0001;
			yaw += yawOffset;
			// if(yaw <= 0){
			// 	yaw = -yaw;
			// }
			// else if(yaw > 0){
			// 	yaw = (M_PI*2 - yaw);
			// }
			if (yaw - lastYaw > M_PI)
				yawRound--;
			else if (yaw - lastYaw < -M_PI)
				yawRound++;
			
			yaw = flitter(0.2, yaw, flitterLastYaw);
			lastYaw = yaw;
			//imuData.yaw = yaw + yawRound * 2 * M_PI; //imuData.yaw为累计转角，往右减小，往左增大
			imuData.yaw = yaw; //imuData.yaw为累计转角，往右减小，往左增大
			imuData.xgravity = gravity.x;
			imuData.ygravity = gravity.y;
			imuData.zgravity = gravity.z;
			// Serial.println(imuData.yaw);
			//获取陀螺仪Z轴加速度
			VectorInt16 rawAccel;
			mpu.dmpGetAccel(&rawAccel, fifoBuffer);
			VectorInt16 accel;
			// 已减去重力分量
			mpu.dmpGetLinearAccel(&accel, &rawAccel, &gravity);
			//imuData.zAccel = accel.z / 8192.0f * 9.8f;

			imuData.yAccel = flitter(0.2, accel.y / 8192.000f, yAccelP);

			// imuData.zAccel = flitter(0.2, rawAccel.z / 8192.000f, zAccelP);
			
			//imuData.zAccel = (accel.z+1766) / 4096.0f * 9.8f;

			//-----------------自设值----------------------------
			// imuData.zAccel = 9.8f;
			

			// Serial.print("test:");
			// Serial.print("  ");
			// Serial.print(q.w, 3);
			// Serial.print("  ");
			// Serial.print(q.x, 3);
			// Serial.print("  ");
			// Serial.print(q.y, 3);
			// Serial.print("  ");
			// Serial.print(q.z, 3);
			// Serial.print("  ");
			// Serial.println(imuData.ygravity, 3);
			//VectorInt16 vec =  minusGravity(rawAccel);
			// float ch[5] = {q.w, q.x, q.y, q.z, imuData.ygravity};
			// vofaPrint((char*)ch,5);
			// Serial.println(vec.z);
		}
		vTaskDelayUntil(&xLastWakeTime, 5); //5ms轮询一次
	}
}

//陀螺仪模块初始化
void IMU_Init()
{
	//初始化IIC
	Wire.begin(5, 4);
	Wire.setClock(400000);

	//初始化陀螺仪
	mpu.initialize();
	Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
	Serial.println("Address: 0x" + String(mpu.getDeviceID(), HEX));
	mpu.setFullScaleAccelRange(MPU6050_IMU::MPU6050_ACCEL_FS_2);
	

    while(mpu.dmpInitialize() != 0){}

	// mpu.setXAccelOffset(-668);
	// mpu.setYAccelOffset(-1632);
	// mpu.setZAccelOffset(998);

	// mpu.setXAccelOffset(-1468);
	// mpu.setYAccelOffset(1847);
	// mpu.setZAccelOffset(1718);

	// mpu.setXGyroOffset(283);
	// mpu.setYGyroOffset(80);
	// mpu.setZGyroOffset(42);
	// mpu.CalibrateAccel(6); //测量偏移数据
	// mpu.CalibrateGyro(6);
	// mpu.PrintActiveOffsets();
	mpu.setDMPEnabled(true);

	//开启陀螺仪数据获取任务
	xTaskCreate(IMU_Task, "IMU_Task", 2048, NULL, 3, NULL);
}

float flitter(float rato, float value, float &last_value)
{

	value = value *  rato + last_value * (1 - rato);
	value = int(value * 1000)/1000.0;
	last_value = value;
	return value;
}

VectorInt16 minusGravity(VectorInt16 vec)
{


 float x = vec.x, y = vec.y, z = 0;
 vec.x -= x * cos(imuData.yaw) - y * sin(imuData.yaw) ;
 vec.y -= x * sin(imuData.yaw) - y * cos(imuData.yaw) ;

 x = vec.x; z = vec.z;
 vec.x -=  x * cos(imuData.pitch) + z * sin(imuData.pitch) ;
 vec.z -= - x * sin(imuData.pitch) + z * cos(imuData.pitch) ;

 y = vec.y, z = vec.z;
 vec.y -=  y * cos(imuData.roll) - z * sin(imuData.roll);
 vec.z -=  y * sin(imuData.roll) + z * cos(imuData.roll);


 return vec;
}