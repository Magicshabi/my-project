#include "Ble_Ctrl.h"

BlePort_t ble_port;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      ble_port.handle_connection_change(true);
      Serial.println(">>> BLE Device Connected <<<"); // 打印连接状态
    };

    void onDisconnect(BLEServer* pServer) {
      ble_port.handle_connection_change(false);
      Serial.println(">>> BLE Device Disconnected <<<"); // 打印断开状态
      pServer->getAdvertising()->start();
    }
};
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        // --- 新增：打印接收到的原始数据 ---
        // Serial.print("[BLE RX Raw]: ");
        // for (int i = 0; i < rxValue.length(); i++) {
        //     Serial.printf("%02X ", (uint8_t)rxValue[i]); // 以16进制打印
        // }
        // Serial.println();
        // ------------------------------------

        // 将数据喂给协议解析器
        ble_port.protocol.parsing((uint8_t*)rxValue.c_str(), rxValue.length());
      }
    }
};
void BlePort_t::begin(const char* localName) {
    BLEDevice::init(localName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
                                        CHARACTERISTIC_UUID_TX,
                                        BLECharacteristic::PROPERTY_NOTIFY
                                    );
    pTxCharacteristic->addDescriptor(new BLE2902());

    pRxCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_WRITE_NR
                                     );
    pRxCharacteristic->setCallbacks(new MyCallbacks());

    pService->start();
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); 
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();
    
    protocol.begin();
    Serial.println("BLE Init OK, Advertising...");
}

void BlePort_t::register_ops_callback(ProtocolSuccessCallback cb) {
    protocol.register_success_callback(cb);
}

void BlePort_t::register_connection_callback(BleConnectionCallback cb) {
    connectionCallback = cb;
}

void BlePort_t::handle_connection_change(bool connected) {
    deviceConnected = connected;
    if (connectionCallback) {
        connectionCallback(connected);
    }
}

void BlePort_t::sendData(uint8_t *data, size_t len) {
    if (deviceConnected) {
        pTxCharacteristic->setValue(data, len);
        pTxCharacteristic->notify();
    }
}
// void BlePort_t::register_ops_callback(ProtocolSuccessCallback cb) {
//     protocol.register_success_callback(cb);
// }

// void BlePort_t::sendData(uint8_t *data, size_t len) {
//     if (deviceConnected) {
//         pTxCharacteristic->setValue(data, len);
//         pTxCharacteristic->notify();
//     }
// }
