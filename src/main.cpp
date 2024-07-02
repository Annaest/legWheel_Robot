/* main.c
 * �������ļ�������������Ҫ�����߼�
 * */

#include "config.h"
#include "can.h"
#include "motor.h"
#include "ble.h"
#include "imu.h"
#include "legs.h"
#include "control.h"
/*******************�궨��*******************/

//��ʱ���ߣ������������ڻ򾭹�ʱ��
uint32_t timeCnt[6] = {0}, lastTimeCnt[6] = {0}, passTimeCnt[6] = {0};
#define TIME_COUNTER(num) do{timeCnt[num] = micros()-lastTimeCnt[num]; lastTimeCnt[num] = micros();}while(0)
#define PASS_TIME_START(num) uint32_t passTimeStart##num = micros()
#define PASS_TIME_STOP(num) passTimeCnt[num] = micros() - passTimeStart##num

float motorOutRatio = 1.0f; //��������ѹ�����������е��ͬʱ��Ч
float sourceVoltage = 12; //��ǰ��Դ��ѹ

/*******************��������*******************/

void setup();
void loop();
void ADC_Task(void *pvParameters);
void ADC_Init();

/*******************��ģ�麯������*******************/



//ADC��ʱ�������񣬲�����ص�ѹ���������������
void ADC_Task(void *pvParameters)
{
	while (1)
	{
		sourceVoltage = analogRead(0) / 4095.0f * 3.3f * 11 * 12 / 13.5f;
		// Serial.print("��ص�ѹ��  ");
		// Serial.println(sourceVoltage);
		if(sourceVoltage < 8)
			sourceVoltage = 12;
		motorOutRatio = (12 - sourceVoltage) / 10.0f + 0.7f;
		vTaskDelay(100);
	}
}

//ADCģ���ʼ��
void ADC_Init()
{
	xTaskCreate(ADC_Task, "ADC_Task", 2048, NULL, 3, NULL);
}

/*******************������*******************/


void setup()
{

	pinMode(LED, OUTPUT); //LED����
	Serial.begin(115200);

	//�ϵ��ȴ�3s
	digitalWrite(LED, HIGH);
	vTaskDelay(3000);
	digitalWrite(LED, LOW);

	//��ʼ������ģ��
	
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

//��������ں���������setup��loop��ģ��Arduino�ṹ
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