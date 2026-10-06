// #ifndef __ESPNOW_CTRL_H__
// #define __ESPNOW_CTRL_H__

// #include "Arduino.h"
// #include <esp_now.h>
// #include <WiFi.h>
// #include <esp_wifi.h> 
// #include "CommProtocol.h"

// #define DEFAULT_ESPNOW_CHANNEL 2

// class EspNow_Ctrl_t {
// public:
//     CommProtocol_t protocol;
//     uint8_t current_channel;

//     void begin(uint8_t channel = DEFAULT_ESPNOW_CHANNEL);
//     void register_ops_callback(ProtocolSuccessCallback cb);
//     void set_channel(uint8_t new_channel);
//     String getMacAddress();

// private:
//     static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
// };

// extern EspNow_Ctrl_t espnow_ctrl;

// #endif
#ifndef __ESPNOW_CTRL_H__
#define __ESPNOW_CTRL_H__

#include "Arduino.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> 
#include "CommProtocol.h"
#include "HX_30HM.h" 

#define DEFAULT_ESPNOW_CHANNEL 2

typedef struct __attribute__((packed)) {
    uint32_t seq;
    int16_t pos[6]; 
} ArmPacket_t;

class EspNow_Ctrl_t {
public:
    CommProtocol_t protocol;
    uint8_t current_channel;
    
    WiFiServer server;
    WiFiClient client;

    bool is_espnow_running = false;

    volatile bool sync_enabled = false; 
    uint16_t soft_start_counter = 0;   
    
    bool is_unicast_mode = false;
    bool auto_bound = false;
    uint8_t target_mac[6];

    EspNow_Ctrl_t();

    bool begin(uint8_t channel = DEFAULT_ESPNOW_CHANNEL); 
    void stop(); 
    
    void startAP(const char* ssid, const char* password);
    void server_loop(); 

    void register_ops_callback(ProtocolSuccessCallback cb);
    void set_channel(uint8_t new_channel);
    void set_peering_mac(const uint8_t* mac);
    void sendData(uint8_t *data, size_t len);
    void mapMasterTeachToFollower(const int16_t *master_positions, int16_t *target_positions);
    void applyMasterTeachPositions(const int16_t *master_positions, int16_t speed = 0);
    
    String getMacAddress();
    String getBoundMasterMac();
    
    void enable_sync(bool en) { 
        sync_enabled = en; 
        if(en) soft_start_counter = 300; 
    }

    volatile int16_t last_target_pos[6];
    volatile bool has_target_data;

private:
    static void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len);
};

extern EspNow_Ctrl_t espnow_ctrl;

#endif