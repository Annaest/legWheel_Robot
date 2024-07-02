#ifndef __BLE_H__
#define __BLE_H__

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

//������UUID���豸����
#define UUID_SERVICE "4c9a0001-fb2e-432e-92ec-c6f5316b4689" //����UUID
#define UUID_RX_CHARA "4c9a0002-fb2e-432e-92ec-c6f5316b4689" //��������UUID
#define UUID_TX_CHARA "4c9a0003-fb2e-432e-92ec-c6f5316b4689" //��������UUID
#define BLE_NAME "BalanceBot"

void BT_SendSampleTask(void *arg);
void BT_Init();

#endif
