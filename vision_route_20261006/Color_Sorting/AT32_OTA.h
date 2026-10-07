#ifndef AT32_OTA_H
#define AT32_OTA_H

#include "Arduino.h"
#include "CommProtocol.h"
#include "Nex_Arm_Board.h"
#include "at32_firmware.h"

class AT32_OTA {
public:
    static bool check_and_update(CommProtocol_t &protocol, HardwareSerial &uart, OLED_t &oled);
    static bool query_version(CommProtocol_t &protocol, HardwareSerial &uart, uint8_t ver[3]);

private:
    static void oled_msg(OLED_t &oled, const char *l0, const char *l1, const char *l2, const char *l3);
};

#endif
