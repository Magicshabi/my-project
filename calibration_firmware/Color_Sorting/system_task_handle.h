#ifndef __SYSTEM_TASK_HANDLE_H__
#define __SYSTEM_TASK_HANDLE_H__

#include "esp_event.h"
#include "CommProtocol.h"

#define FIRMWARE_VERSION_MAJOR       1
#define FIRMWARE_VERSION_MINOR       0
#define FIRMWARE_VERSION_PATCH       2
#define FIRMWARE_VERSION_STR         "1.0.2-pose-cal"
#define TIMER_PERIOD                10

enum TimingEvent {
    TIMING_EVENT_BUZZER_UPDATE = 0,
    TIMING_EVENT_SERVO_STATUS_UPDATE,
    TIMING_EVENT_BAT_UPDATE,
    TIMING_EVENT_OLED_REFRESH,
    TIMING_EVENT_SERVO_SYNC
};

enum TimingEventPeriod {
    BUZZER_UPDATE_PERIOD = 20,
    SERVO_STATUS_UPDATE_PERIOD = 500,
    BAT_UPDATE_PERIOD = 200,
    OLED_REFRESH_PERIOD = 500,
    SERVO_SYNC_PERIOD = 10
};

enum StatusEvent {
    STATUS_EVENT_BOOT_BUTTON = 0,
    STATUS_EVENT_USER_BUTTON,
    STATUS_EVENT_ERROR,
};

enum CtrlMode {
    UART_MODE = 0,
    WIFI_MODE,
    BLE_MODE
};

void register_system_task(esp_event_loop_handle_t *event_loop);
void system_loop_handler(void);
void at32_packet_callback(PacketTypeDef* rx_packet);
void pump_at32_feedback(void);
bool sync_arm_feedback(uint32_t timeout_ms);
bool get_last_servo_positions(int16_t out_pos[6]);
bool sync_teach_handle_master_packet(uint32_t seq, const int16_t master_positions[6]);

extern uint8_t global_acc;
extern uint8_t global_channel;

#endif
