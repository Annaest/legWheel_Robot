/* main.c
 * 主代码文件，包含所有主要代码逻辑
 * */

#include "config.h"
#include "can.h"
#include "motor.h"
#include "ble.h"
#include "imu.h"
#include "legs.h"
#include "control.h"
/*******************宏定义*******************/

//计时工具，计算运行周期或经过时间
uint32_t timeCnt[6] = {0}, lastTimeCnt[6] = {0}, passTimeCnt[6] = {0};
#define TIME_COUNTER(num) do{timeCnt[num] = micros()-lastTimeCnt[num]; lastTimeCnt[num] = micros();}while(0)
#define PASS_TIME_START(num) uint32_t passTimeStart##num = micros()
#define PASS_TIME_STOP(num) passTimeCnt[num] = micros() - passTimeStart##num

float motorOutRatio = 1.0f; //电机输出电压比例，对所有电机同时有效
float sourceVoltage = 12; //当前电源电压

/*******************函数声明*******************/

void setup();
void loop();
void ADC_Task(void *pvParameters);
void ADC_Init();

/*******************各模块函数定义*******************/



//ADC定时采样任务，采样电池电压并计算电机输出比例
void ADC_Task(void *pvParameters)
{
	while (1)
	{
		sourceVoltage = analogRead(0) / 4095.0f * 3.3f * 11 * 12 / 13.5f;
		// Serial.print("电池电压：  ");
		// Serial.println(sourceVoltage);
		if(sourceVoltage < 8)
			sourceVoltage = 12;
		motorOutRatio = (12 - sourceVoltage) / 10.0f + 0.7f;
		vTaskDelay(100);
	}
}

//ADC模块初始化
void ADC_Init()
{
	xTaskCreate(ADC_Task, "ADC_Task", 2048, NULL, 3, NULL);
}

/*******************主函数*******************/


void setup()
{

	pinMode(LED, OUTPUT); //LED引脚
	Serial.begin(115200);

	//上电后等待3s
	digitalWrite(LED, HIGH);
	vTaskDelay(3000);
	digitalWrite(LED, LOW);

	//初始化所有模块
	
	ADC_Init();
	CAN_Init();
	IMU_Init();
	Motor_InitAll();
	BT_Init();
	Leg_InitAll();
	Ctrl_Init();
	uint8_t startFram = 255;
	CAN_SendFrame(0x100, &startFram);
}

void loop()
{
	
}

//真正的入口函数，调用setup和loop，模仿Arduino结构
extern "C" void app_main()
{
	//esp_log_level_set("*", ESP_LOG_NONE );
	setup();
	while (1)
	{
		loop();
	}
}

void vofaPrint(char *str, int num)
{
	char tail[4] = {0x00,0x00,0x80,0x7f};
	Serial.write(str,sizeof(float)*num);
	Serial.write(tail, 4);
}