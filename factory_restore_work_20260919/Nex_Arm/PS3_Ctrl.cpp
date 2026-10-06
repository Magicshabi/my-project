#include "PS3_Ctrl.h"

PS3Port_t ps3_port;

// 静态回调函数指针
void (*PS3Port_t::button_callback)(int button, bool pressed) = nullptr;
void (*PS3Port_t::analog_callback)(int stick, int8_t x, int8_t y) = nullptr;

void PS3Port_t::onConnect() {
    ps3_port.deviceConnected = true;
    Serial.println(">>> PS3 Controller Connected <<<");
    
    // 设置LED灯表示已连接
    Ps3.setPlayer(1);
}

void PS3Port_t::onDisconnect() {
    ps3_port.deviceConnected = false;
    Serial.println(">>> PS3 Controller Disconnected <<<");
}

void PS3Port_t::onEvent() {
    // 处理按键事件
    if (button_callback) {
        // 检查所有按键状态变化
        if (Ps3.event.button_down.cross) button_callback(0, true);
        if (Ps3.event.button_up.cross) button_callback(0, false);
        
        if (Ps3.event.button_down.circle) button_callback(1, true);
        if (Ps3.event.button_up.circle) button_callback(1, false);
        
        if (Ps3.event.button_down.square) button_callback(2, true);
        if (Ps3.event.button_up.square) button_callback(2, false);
        
        if (Ps3.event.button_down.triangle) button_callback(3, true);
        if (Ps3.event.button_up.triangle) button_callback(3, false);
        
        if (Ps3.event.button_down.l1) button_callback(4, true);
        if (Ps3.event.button_up.l1) button_callback(4, false);
        
        if (Ps3.event.button_down.r1) button_callback(5, true);
        if (Ps3.event.button_up.r1) button_callback(5, false);
        
        if (Ps3.event.button_down.l2) button_callback(6, true);
        if (Ps3.event.button_up.l2) button_callback(6, false);
        
        if (Ps3.event.button_down.r2) button_callback(7, true);
        if (Ps3.event.button_up.r2) button_callback(7, false);
        
        if (Ps3.event.button_down.select) button_callback(8, true);
        if (Ps3.event.button_up.select) button_callback(8, false);
        
        if (Ps3.event.button_down.start) button_callback(9, true);
        if (Ps3.event.button_up.start) button_callback(9, false);
        
        if (Ps3.event.button_down.l3) button_callback(10, true);
        if (Ps3.event.button_up.l3) button_callback(10, false);
        
        if (Ps3.event.button_down.r3) button_callback(11, true);
        if (Ps3.event.button_up.r3) button_callback(11, false);
        
        if (Ps3.event.button_down.up) button_callback(12, true);
        if (Ps3.event.button_up.up) button_callback(12, false);
        
        if (Ps3.event.button_down.down) button_callback(13, true);
        if (Ps3.event.button_up.down) button_callback(13, false);
        
        if (Ps3.event.button_down.left) button_callback(14, true);
        if (Ps3.event.button_up.left) button_callback(14, false);
        
        if (Ps3.event.button_down.right) button_callback(15, true);
        if (Ps3.event.button_up.right) button_callback(15, false);
        
        if (Ps3.event.button_down.ps) button_callback(16, true);
        if (Ps3.event.button_up.ps) button_callback(16, false);
    }
    
    // 处理摇杆事件（只在有变化时触发）
    if (analog_callback) {
        static int8_t last_lx = 0, last_ly = 0, last_rx = 0, last_ry = 0;
        
        int8_t lx = Ps3.data.analog.stick.lx;
        int8_t ly = Ps3.data.analog.stick.ly;
        int8_t rx = Ps3.data.analog.stick.rx;
        int8_t ry = Ps3.data.analog.stick.ry;
        
        // 左摇杆
        if (lx != last_lx || ly != last_ly) {
            analog_callback(0, lx, ly);
            last_lx = lx;
            last_ly = ly;
        }
        
        // 右摇杆
        if (rx != last_rx || ry != last_ry) {
            analog_callback(1, rx, ry);
            last_rx = rx;
            last_ry = ry;
        }
    }
}

void PS3Port_t::begin(const char* mac_address) {
    // 初始化PS3控制器
    Ps3.attach(onEvent);
    Ps3.attachOnConnect(onConnect);
    Ps3.attachOnDisconnect(onDisconnect);
    
    // 开始监听 - 如果MAC地址为空或全0，则接受任何手柄
    if(mac_address == nullptr || strcmp(mac_address, "00:00:00:00:00:00") == 0) {
        Ps3.begin();  // 不指定MAC，接受任何手柄
        Serial.println("PS3 Controller Init OK");
        Serial.println("Waiting for ANY PS3 controller...");
    } else {
        Ps3.begin(mac_address);  // 指定MAC地址
        Serial.println("PS3 Controller Init OK");
        Serial.print("Waiting for PS3 controller with MAC: ");
        Serial.println(mac_address);
    }
}

void PS3Port_t::register_button_callback(void (*callback)(int button, bool pressed)) {
    button_callback = callback;
}

void PS3Port_t::register_analog_callback(void (*callback)(int stick, int8_t x, int8_t y)) {
    analog_callback = callback;
}

bool PS3Port_t::isConnected() {
    return deviceConnected && Ps3.isConnected();
}

void PS3Port_t::setRumble(uint8_t small_motor, uint8_t large_motor) {
    if (isConnected()) {
        Ps3.setRumble(small_motor, large_motor);
    }
}

void PS3Port_t::setLed(uint8_t led_num) {
    if (isConnected()) {
        Ps3.setPlayer(led_num);
    }
}
