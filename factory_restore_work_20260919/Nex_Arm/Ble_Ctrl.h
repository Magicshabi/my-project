#ifndef __BLE_CTRL_H__
#define __BLE_CTRL_H__

#include "Arduino.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "CommProtocol.h"

// Nordic UART Service UUIDs
#define SERVICE_UUID           "0000ffe0-0000-1000-8000-00805f9b34fb" 
#define CHARACTERISTIC_UUID_RX "0000ffe1-0000-1000-8000-00805f9b34fb"
#define CHARACTERISTIC_UUID_TX "0000ffe2-0000-1000-8000-00805f9b34fb"

using BleConnectionCallback = std::function<void(bool connected)>;

class BlePort_t {
public:
    CommProtocol_t protocol;
    volatile bool deviceConnected = false;

    void begin(const char* localName);
    void register_ops_callback(ProtocolSuccessCallback cb);
    void register_connection_callback(BleConnectionCallback cb);
    void handle_connection_change(bool connected);
    void sendData(uint8_t *data, size_t len);

private:
    BLEServer *pServer = NULL;
    BLECharacteristic *pTxCharacteristic = NULL;
    BLECharacteristic *pRxCharacteristic = NULL;
    BleConnectionCallback connectionCallback;
};

extern BlePort_t ble_port;

#endif
