#include "EspNow_Ctrl.h"
#include "HX_30HM.h"
#include "Global.h"
#include "Robot_Arm.h"
#include "system_task_handle.h"
#include "esp_coexist.h"

EspNow_Ctrl_t espnow_ctrl;

#define MASK_HOST(x, bit)  ((x) < 0 ? (-x) | (1U) << (bit): (x))

EspNow_Ctrl_t::EspNow_Ctrl_t() : server(8080) {
    auto_bound = false;
    is_unicast_mode = false;
    memset(target_mac, 0, sizeof(target_mac));
    memset((void*)last_target_pos, 0, sizeof(last_target_pos));
    has_target_data = false;
}

void EspNow_Ctrl_t::OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    if (espnow_ctrl.is_unicast_mode) {
        if (memcmp(mac, espnow_ctrl.target_mac, 6) != 0) return;
    }

    bool looks_like_protocol = (len >= 3 &&
                                incomingData[0] == 0xFF &&
                                incomingData[1] == 0xFF &&
                                incomingData[2] == 0xFF);

    if (len == sizeof(ArmPacket_t) && !looks_like_protocol) {
        ArmPacket_t *pkt = (ArmPacket_t*)incomingData;
        if (sync_teach_handle_master_packet(pkt->seq, pkt->pos)) {
            return;
        }

        if (!espnow_ctrl.sync_enabled) {
            return;
        }

        int16_t current_speed = 0;
        if (espnow_ctrl.soft_start_counter > 0) espnow_ctrl.soft_start_counter--;

        espnow_ctrl.applyMasterTeachPositions(pkt->pos, current_speed);
    }
    else {
        espnow_ctrl.protocol.parsing((uint8_t*)incomingData, len);
    }
}

void EspNow_Ctrl_t::mapMasterTeachToFollower(const int16_t *master_positions, int16_t *target_positions) {
    for (int i = 0; i < 6; i++) {
        int id = i + 1;
        int16_t master_pos = master_positions[i];

        if (id == 2) {
            target_positions[i] = (int16_t)constrain(4096 - (int32_t)master_pos, 0, 4096);
        } else if (id == 6) {
            int32_t delta = (int32_t)master_pos - 2048;
            target_positions[i] = (int16_t)constrain(2833 + delta * 4, 1195, 2833);
        } else {
            target_positions[i] = master_pos;
        }
    }
}

extern volatile bool servo_uart_busy;

void EspNow_Ctrl_t::applyMasterTeachPositions(const int16_t *master_positions, int16_t speed) {
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    int16_t target_positions[6];

    mapMasterTeachToFollower(master_positions, target_positions);

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;
    servo.sync_write_pos_speed(ids, target_positions, speed, 6);
    servo_uart_busy = false;

    for (int i = 0; i < 6; i++) {
        last_target_pos[i] = target_positions[i];
    }
    has_target_data = true;
}

bool EspNow_Ctrl_t::begin(uint8_t channel) {
    if (client) client.stop();
    server.end();

    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(100);

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(20);

    current_channel = channel;

    esp_err_t init_ret = esp_now_init();
    if (init_ret != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        is_espnow_running = false;
        return false;
    }
    is_espnow_running = true;
    auto_bound = false;
    is_unicast_mode = false;
    memset(target_mac, 0, 6);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(current_channel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    esp_now_register_recv_cb(OnDataRecv);

    protocol.begin();
    return true;
}

void EspNow_Ctrl_t::stop() {
    if (is_espnow_running) {
        esp_now_unregister_recv_cb();
        esp_now_deinit();
        is_espnow_running = false;
    }
    auto_bound = false;
    is_unicast_mode = false;
    memset(target_mac, 0, 6);

    if (client) client.stop();
    server.end();

    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
}

void EspNow_Ctrl_t::startAP(const char* ssid, const char* password) {
    if (is_espnow_running) {
        esp_now_unregister_recv_cb();
        esp_now_deinit();
        is_espnow_running = false;
    }
    esp_wifi_set_promiscuous(false);

    WiFi.disconnect(true);
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(300);

    WiFi.mode(WIFI_AP);
    WiFi.persistent(false);

    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);

    delay(100);

    bool result = WiFi.softAP(ssid, password, 6, 0, 4);

    if(result) {
        delay(200);

        esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);

        esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);

        esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
        server.begin();
        Serial.printf("[WiFi AP] Started: %s IP: %s (ch6, 11g/n)\n", ssid, WiFi.softAPIP().toString().c_str());
    } else {
        Serial.println("[WiFi AP] softAP FAILED, retrying on ch11...");
        delay(500);

        result = WiFi.softAP(ssid, password, 11, 0, 4);
        if(result) {
            delay(200);
            esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
            esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
            esp_coex_preference_set(ESP_COEX_PREFER_BALANCE);
            server.begin();
            Serial.printf("[WiFi AP] Retry OK: %s IP: %s (ch11)\n", ssid, WiFi.softAPIP().toString().c_str());
        } else {
            Serial.println("[WiFi AP] softAP FAILED after retry");
        }
    }
}
void EspNow_Ctrl_t::server_loop() {
    if (server.hasClient()) {
        if (client && client.connected()) {
            WiFiClient newClient = server.available();
            newClient.stop();
        } else {
            client = server.available();
        }
    }

    if (client && client.connected()) {
        if(client.available()) {
            uint8_t buf[256];
            int len = client.read(buf, 255);
            if(len > 0) {
                protocol.parsing(buf, len);
            }
        }
    }
}

void EspNow_Ctrl_t::set_peering_mac(const uint8_t* mac) {
    bool all_zero = true;
    for(int i=0; i<6; i++) {
        if(mac[i] != 0) all_zero = false;
    }
    if (all_zero) {
        is_unicast_mode = false;
        auto_bound = false;
        memset(target_mac, 0, 6);
    } else {
        memcpy(target_mac, mac, 6);
        is_unicast_mode = true;
        auto_bound = true;
    }
}

void EspNow_Ctrl_t::register_ops_callback(ProtocolSuccessCallback cb) {
    protocol.register_success_callback(cb);
}

void EspNow_Ctrl_t::set_channel(uint8_t new_channel) {
    if(new_channel < 1 || new_channel > 13) return;
    current_channel = new_channel;

    if(WiFi.getMode() == WIFI_STA) {
        esp_wifi_set_promiscuous(true);
        esp_wifi_set_channel(current_channel,
        WIFI_SECOND_CHAN_NONE);
        esp_wifi_set_promiscuous(false);
    }
}
void EspNow_Ctrl_t::sendData(uint8_t *data, size_t len) {
    if (WiFi.getMode() == WIFI_AP && client && client.connected()) {
        client.write(data, len);
    }
}
String EspNow_Ctrl_t::getMacAddress() {
    return WiFi.macAddress();
}

String EspNow_Ctrl_t::getBoundMasterMac() {
    if (!auto_bound) return "---";
    char buf[18];
    snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
        target_mac[0], target_mac[1], target_mac[2],
        target_mac[3], target_mac[4], target_mac[5]);
    return String(buf);
}
