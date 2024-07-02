#include "ble.h"
#include "config.h"
#include "control.h"
#include "imu.h"
//蓝牙相关全局对象指针
BLECharacteristic *chara_tx, *chara_rx;
BLEServer *server;
BLEService *service;

/******* 蓝牙模块 *******/

//通过蓝牙notify发送数据
extern "C" void BT_Send(uint8_t *data, uint32_t len)
{
	chara_tx->setValue(data, len);
	chara_tx->notify();
}



//重写蓝牙服务回调类，处理连接和断连事件
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

//重写蓝牙特征回调类，处理特征读写事件
class MyBleCharaCallbacks : public BLECharacteristicCallbacks
{
public:
	void onWrite(BLECharacteristic *chara) //特征写入事件，收到蓝牙数据
	{
		uint32_t len = chara->getLength();
		uint8_t *data = chara->getData();
		

		// Serial.print("onWrite: " + String(len) + " bytes: "); //测试蓝牙收发
		// for (int i = 0; i < len; i++)
		// 	Serial.print(String(data[i], HEX) + " ");
		// Serial.println();
		// BT_Send(data, len);
		// 速度遥控
		speed[0] = (data[2] - 100) / 15.0;
		speed[1] = (data[2] - 100) / 15.0;
		// 方向遥控
		if(fabs(data[1] - 100) > 20)
			targetYaw += (data[1] - 100) / 500.0;
		// if(targetYaw >= M_PI*2) 	targetYaw = 0;
		// if(targetYaw < 0 )   targetYaw = M_PI*2 + targetYaw;
		setLegPos = -data[3]/300.0 + 1.5;
		// Serial.println(setLegPos);
	}
};

//蓝牙模块初始化，创建设备、服务、特征并启动广播
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
