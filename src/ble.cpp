#include "ble.h"
#include "config.h"
#include "control.h"
#include "imu.h"
//�������ȫ�ֶ���ָ��
BLECharacteristic *chara_tx, *chara_rx;
BLEServer *server;
BLEService *service;

/******* ����ģ�� *******/

//ͨ������notify��������
extern "C" void BT_Send(uint8_t *data, uint32_t len)
{
	chara_tx->setValue(data, len);
	chara_tx->notify();
}



//��д��������ص��࣬�������ӺͶ����¼�
class MyBleServerCallbacks : public BLEServerCallbacks
{
public:
	void onConnect(BLEServer *server)
	{
		Serial.println("onConnect");
		server->getAdvertising()->stop();
	}
	void onDisconnect(BLEServer *server)
	{
		Serial.println("onDisconnect");
		server->getAdvertising()->start();
	}
};

//��д���������ص��࣬����������д�¼�
class MyBleCharaCallbacks : public BLECharacteristicCallbacks
{
public:
	void onWrite(BLECharacteristic *chara) //����д���¼����յ���������
	{
		uint32_t len = chara->getLength();
		uint8_t *data = chara->getData();
		

		// Serial.print("onWrite: " + String(len) + " bytes: "); //���������շ�
		// for (int i = 0; i < len; i++)
		// 	Serial.print(String(data[i], HEX) + " ");
		// Serial.println();
		// BT_Send(data, len);
		// �ٶ�ң��
		speed[0] = (data[2] - 100) / 15.0;
		speed[1] = (data[2] - 100) / 15.0;
		// ����ң��
		if(fabs(data[1] - 100) > 20)
			targetYaw += (data[1] - 100) / 500.0;
		// if(targetYaw >= M_PI*2) 	targetYaw = 0;
		// if(targetYaw < 0 )   targetYaw = M_PI*2 + targetYaw;
		setLegPos = -data[3]/300.0 + 1.5;
		// Serial.println(setLegPos);
	}
};

//����ģ���ʼ���������豸�����������������㲥
void BT_Init()
{
	BLEDevice::init(BLE_NAME);
	server = BLEDevice::createServer();
	server->setCallbacks(new MyBleServerCallbacks());
	service = server->createService(UUID_SERVICE);
	chara_tx = service->createCharacteristic(UUID_TX_CHARA, BLECharacteristic::PROPERTY_NOTIFY);
	chara_tx->addDescriptor(new BLE2902());
	chara_rx = service->createCharacteristic(UUID_RX_CHARA, BLECharacteristic::PROPERTY_WRITE_NR);
	chara_rx->setCallbacks(new MyBleCharaCallbacks());
	service->start();
	server->getAdvertising()->start();
}
