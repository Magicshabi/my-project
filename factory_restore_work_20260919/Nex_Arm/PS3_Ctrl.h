#ifndef __PS3_CTRL_H__
#define __PS3_CTRL_H__

#include "Arduino.h"
#include <Ps3Controller.h>

// PS3手柄控制类
class PS3Port_t {
public:
    bool deviceConnected = false;
    
    // 初始化PS3手柄，传入MAC地址
    void begin(const char* mac_address);
    
    // 注册按键事件回调
    void register_button_callback(void (*callback)(int button, bool pressed));
    
    // 注册摇杆事件回调
    void register_analog_callback(void (*callback)(int stick, int8_t x, int8_t y));
    
    // 获取当前连接状态
    bool isConnected();
    
    // 设置手柄震动 (0-255)
    void setRumble(uint8_t small_motor, uint8_t large_motor);
    
    // 设置LED灯 (1-4)
    void setLed(uint8_t led_num);

private:
    static void onConnect();
    static void onDisconnect();
    static void onEvent();
    
    static void (*button_callback)(int button, bool pressed);
    static void (*analog_callback)(int stick, int8_t x, int8_t y);
};

extern PS3Port_t ps3_port;

#endif
