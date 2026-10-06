#include "system_task_handle.h"
#include "Global.h"
#include "Robot_Arm.h"
#include "usb_ctrl.h"
#include "Ble_Ctrl.h"
#include "PS3_Ctrl.h"
#include "EspNow_Ctrl.h"
#include <Preferences.h>
#include "FaceTracker.h"
#include "SelfLearningTracker.h"
#include "AprilTagTracker.h"
#include "AprilTagGrabber.h"
#include "ColorTrackerRot.h"
#include "ColorGrabber.h"
#include "ColorGrabber.h"
#include "AiLLMControl.h"
#include "GarbageGrabber.h"
#include "CalibrationGrabber.h"
#include "GestureTracker.h"
#include "AT32_OTA.h"
#include <LittleFS.h>

static const char* TAG = "system_task";
static TimerHandle_t TIMER;
static esp_event_loop_handle_t loop_with_sys_task;
static CommProtocol_t at32_protocol;
static bool is_downloading = false;
static Preferences prefs;
static int16_t g_last_servo_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
static uint32_t g_last_servo_feedback_ms = 0;
static uint32_t g_last_servo_stream_ms = 0;

enum SystemWorkMode { MODE_ESPNOW_RUNNING, MODE_WIFI_AP_RUNNING };
static SystemWorkMode current_system_mode = MODE_WIFI_AP_RUNNING;

enum OledMode { SHOW_STATUS, SHOW_NET_INFO, SHOW_CUSTOM, SHOW_ICON };
static OledMode current_oled_mode = SHOW_STATUS;

static uint32_t last_display_time = 0;
static uint32_t ps3_prompt_start_time = 0;
static bool ps3_prompt_active = false;
uint8_t global_channel = 2;
uint8_t global_acc = 245;

static uint8_t chassis_type = CHASSIS_NONE;
static float chassis_wheel_diameter = 95.0f;
static float chassis_wheel_base = 195.0f;
static float chassis_track_width = 220.0f;
static uint8_t chassis_motor_dir[4] = {0,0,0,0};
static int8_t chassis_max_speed = 100;

// Version 4 uses the complete six-byte AT32 PID payload, including the
// 16-bit minimum-force field.  Older markers must not suppress it.
static constexpr uint8_t SERVO_PID_CONFIG_VERSION = 4;
static constexpr char SERVO_PID_VERSION_KEY[] = "servo_pid_ver";
static constexpr uint8_t DEFAULT_SERVO_PID_P = 32;
static constexpr uint8_t DEFAULT_SERVO_PID_I = 0;
static constexpr uint8_t DEFAULT_SERVO_PID_D = 32;
static constexpr uint16_t DEFAULT_SERVO_PID_MIN_FORCE = 25;
static constexpr uint16_t SERVO_PID_WRITE_SETTLE_MS = 150;

static portMUX_TYPE ble_chassis_mux = portMUX_INITIALIZER_UNLOCKED;
static bool ble_chassis_active = false;
static bool ble_disconnect_pending = false;

static float arm_kin[9] = {
    110.45f,
    225.00f,
     36.97f,
    145.00f,
      0.0f,
    130.23f,
      0.0f,
     50.0f,
    70.5f
};

static bool lerobotBridgeMode = false;
static int16_t cached_servo_pos[6] = {0};
static bool cached_pos_valid = false;

#define CMD_LR_ENTER      0x60
#define CMD_LR_EXIT       0x61
#define CMD_LR_READ_POS   0x62
#define CMD_LR_WRITE_POS  0x63
#define CMD_LR_TORQUE     0x64
#define CMD_LR_READ_REG   0x65
#define CMD_LR_WRITE_REG  0x66
#define CMD_LR_PING       0x67
#define CMD_SERVO_STREAM  0x68

volatile bool servo_uart_busy = false;

static void send_local_response(uint8_t cmd, const uint8_t* data, uint8_t data_len) {
    uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, cmd, (uint8_t*)data, data_len);
    serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
    ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
    espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
}

void bridge_at32_callback(PacketTypeDef* pkt) {
    uint8_t cmd = pkt->elements.cmd;
    uint8_t data_len = pkt->elements.length - 2;

    if (cmd == 96 || cmd == 97 || cmd == 98) {
        uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, cmd, pkt->elements.args, data_len);
        serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
    }

}

void bridge_usb_callback(PacketTypeDef* pkt) {
    if (!lerobotBridgeMode) return;

    uint8_t cmd = pkt->elements.cmd;
    uint8_t *args = pkt->elements.args;
    uint8_t data_len = pkt->elements.length - 2;
    uint8_t len;

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;

    switch (cmd) {
        case 96: {
            len = at32_protocol.tx_packet_complete(0xFF, 96, nullptr, 0);
            servo.uart->write((const uint8_t*)&at32_protocol.tx_packet, len);

            break;
        }
        case 97: {
            len = at32_protocol.tx_packet_complete(0xFF, 97, args, 12);
            servo.uart->write((const uint8_t*)&at32_protocol.tx_packet, len);
            break;
        }
        case 98: {
            // Forward the CMD_LR_TORQUE frame to the AT32 co-processor as
            // before. The AT32 loops i=1..6 and disables each servo one by
            // one; on rare occasions the last iteration (gripper) is lost.
            len = at32_protocol.tx_packet_complete(0xFF, 98, args, 1);
            servo.uart->write((const uint8_t*)&at32_protocol.tx_packet, len);
            // Also handle it locally by broadcasting REG_TORQUE_ENABLE on
            // the servo bus via the ESP32-side arm.set_torque(). This is
            // the same one-shot broadcast the desktop app uses (CMD 61)
            // and guarantees the gripper is released even if the per-servo
            // loop above misses a frame.
            arm.set_torque(args[0] != 0);
            break;
        }
        case CMD_SET_TORQUE: {
            // Legacy sys-torque command. In LeRobot bridge mode we honour it
            // locally (no AT32 forwarding needed) so that
            // "torque off" always reaches every servo, gripper included.
            if (pkt->elements.length >= 3) {
                arm.set_torque(args[0] != 0);
            }
            break;
        }
        default: {
            len = at32_protocol.tx_packet_complete(0xFF, cmd, args, data_len);
            servo.uart->write((const uint8_t*)&at32_protocol.tx_packet, len);
            break;
        }
    }

    servo_uart_busy = false;
}

void enter_lerobot_bridge_mode() {

    serial_port.protocol.register_success_callback(bridge_usb_callback);

    at32_protocol.register_success_callback(bridge_at32_callback);

    lerobotBridgeMode = true;
}

static BluetoothMode current_bt_mode = BT_MODE_BLE;
static char ps3_mac_buf[18] = "10:00:00:00:85:95";
const char* PS3_MAC = ps3_mac_buf;

const char* AP_SSID = "NexArm";
const char* AP_PASS = "hiwonder";

ESP_EVENT_DEFINE_BASE(SYS_STATUS_EVENTS);
ESP_EVENT_DEFINE_BASE(SYS_TIMING_EVENTS);
void func_ctrl_callback(PacketTypeDef* self);

static bool decode_chassis_command(const PacketTypeDef* packet, bool* moving) {
    if (packet == nullptr || moving == nullptr ||
        packet->elements.id != 0xFF || packet->elements.length < 2) {
        return false;
    }

    const uint8_t data_len = packet->elements.length - 2;
    const uint8_t* args = packet->elements.args;

    switch (packet->elements.cmd) {
        case CMD_SET_SINGLE_MOTOR:
            if (data_len < 2 || args[0] < 1 || args[0] > 4) return false;
            *moving = ((int8_t)args[1] != 0);
            return true;
        case CMD_STOP_ALL_MOTOR:
            if (data_len != 0) return false;
            *moving = false;
            return true;
        case CMD_SET_MOTOR_SPEED:
            if (data_len < 4) return false;
            *moving = ((int8_t)args[0] != 0 || (int8_t)args[1] != 0 ||
                       (int8_t)args[2] != 0 || (int8_t)args[3] != 0);
            return true;
        case CMD_MECANUM_CONTROL:
            if (data_len < 3) return false;
            *moving = ((int8_t)args[0] != 0 || (int8_t)args[1] != 0 ||
                       (int8_t)args[2] != 0);
            return true;
        case CMD_TANK_CONTROL:
            if (data_len < 2) return false;
            *moving = ((int8_t)args[0] != 0 || (int8_t)args[1] != 0);
            return true;
        default:
            return false;
    }
}

static void update_chassis_control_source(const PacketTypeDef* packet, bool from_ble) {
    bool moving = false;
    if (!decode_chassis_command(packet, &moving)) {
        return;
    }

    portENTER_CRITICAL(&ble_chassis_mux);
    if (from_ble) {
        ble_chassis_active = moving;
    } else {
        ble_chassis_active = false;
    }
    portEXIT_CRITICAL(&ble_chassis_mux);
}

static void non_ble_ctrl_callback(PacketTypeDef* packet) {
    update_chassis_control_source(packet, false);
    func_ctrl_callback(packet);
}

static void ble_ctrl_callback(PacketTypeDef* packet) {
    update_chassis_control_source(packet, true);
    func_ctrl_callback(packet);
}

static void ble_connection_callback(bool connected) {
    if (connected) {
        return;
    }

    portENTER_CRITICAL(&ble_chassis_mux);
    ble_disconnect_pending = true;
    portEXIT_CRITICAL(&ble_chassis_mux);
}

static void service_ble_chassis_failsafe() {
    bool should_stop = false;

    portENTER_CRITICAL(&ble_chassis_mux);
    if (ble_disconnect_pending) {
        ble_disconnect_pending = false;
        if (ble_chassis_active) {
            ble_chassis_active = false;
            should_stop = true;
        }
    }
    portEXIT_CRITICAL(&ble_chassis_mux);

    if (should_stop) {
        arm.board.motor.stop();
        Serial.println("[BLE] Chassis stopped: disconnected");
    }
}

static void apply_chassis_config_to_runtime() {
    arm.board.mecanum.configure(chassis_wheel_diameter,
                                chassis_wheel_base,
                                chassis_track_width,
                                chassis_motor_dir,
                                chassis_max_speed);
}

// Wait for one complete AT32 protocol reply.  This is used during boot,
// before the normal at32_protocol callback loop is running.
static bool wait_at32_reply(uint8_t expected_cmd, uint8_t *out_args,
                            uint8_t *out_args_len, uint32_t timeout_ms) {
    uint32_t start = millis();
    uint8_t state = 0;
    uint8_t length = 0;
    uint8_t cmd = 0;
    uint8_t args[MAX_ARGS_SIZE];
    uint8_t arg_count = 0;
    uint16_t checksum = 0;

    while (millis() - start < timeout_ms) {
        if (!servo.uart->available()) {
            delay(1);
            continue;
        }

        const uint8_t byte = servo.uart->read();
        switch (state) {
            case 0:
                state = (byte == FRAME_HEADER_1) ? 1 : 0;
                break;
            case 1:
                state = (byte == FRAME_HEADER_2) ? 2 : 0;
                break;
            case 2:
                checksum = byte;
                state = 3;
                break;
            case 3:
                length = byte;
                checksum += byte;
                arg_count = 0;
                state = (length >= 2 && length <= MAX_ARGS_SIZE) ? 4 : 0;
                break;
            case 4:
                cmd = byte;
                checksum += byte;
                state = (length == 2) ? 6 : 5;
                break;
            case 5:
                args[arg_count++] = byte;
                checksum += byte;
                if (arg_count == length - 2) state = 6;
                break;
            case 6:
                state = 0;
                if (byte == (uint8_t)(~checksum) && cmd == expected_cmd) {
                    if (out_args && out_args_len) {
                        memcpy(out_args, args, arg_count);
                        *out_args_len = arg_count;
                    }
                    return true;
                }
                break;
        }
    }
    return false;
}

static bool read_servo_setting(uint8_t cmd, uint8_t id, uint8_t *reply,
                               uint8_t *reply_len) {
    while (servo.uart->available()) servo.uart->read();

    const uint8_t request[] = {id};
    if (servo.tx_frame_write(0xFF, cmd, request, sizeof(request)) !=
        sizeof(request) + 6) {
        return false;
    }
    return wait_at32_reply(cmd, reply, reply_len, 300);
}

static bool verify_servo_pid(uint8_t id) {
    uint8_t reply[MAX_ARGS_SIZE] = {};
    uint8_t reply_len = 0;
    if (!read_servo_setting(CMD_GET_PID_PARAM, id, reply, &reply_len)) {
        Serial.printf("[PID] Servo %u readback timed out\n", id);
        return false;
    }

    const bool matches = reply_len >= 6 &&
                         reply[0] == id &&
                         reply[1] == DEFAULT_SERVO_PID_P &&
                         reply[2] == DEFAULT_SERVO_PID_I &&
                         reply[3] == DEFAULT_SERVO_PID_D &&
                         reply[4] == (uint8_t)(DEFAULT_SERVO_PID_MIN_FORCE & 0xFF) &&
                         reply[5] == (uint8_t)(DEFAULT_SERVO_PID_MIN_FORCE >> 8);
    if (!matches) {
        Serial.printf("[PID] Servo %u readback mismatch: len=%u data=%u,%u,%u,%u,%u,%u\n",
                      id, reply_len, reply[0], reply[1], reply[2], reply[3],
                      reply[4], reply[5]);
    }
    return matches;
}

static bool write_default_servo_pid() {
    for (uint8_t id = 1; id <= 6; id++) {
        // AT32 command 59 expects [servo_id, P, I, D, min_force_l,
        // min_force_h] in its payload.  The protocol frame itself is
        // addressed to the AT32 (0xFF).
        const uint8_t pid_args[] = {
            id,
            DEFAULT_SERVO_PID_P,
            DEFAULT_SERVO_PID_I,
            DEFAULT_SERVO_PID_D,
            (uint8_t)(DEFAULT_SERVO_PID_MIN_FORCE & 0xFF),
            (uint8_t)(DEFAULT_SERVO_PID_MIN_FORCE >> 8),
        };

        while (servo_uart_busy) { delayMicroseconds(100); }
        servo_uart_busy = true;
        const uint8_t written = servo.tx_frame_write(0xFF, CMD_SET_PID_PARAM,
                                                      pid_args, sizeof(pid_args));
        servo_uart_busy = false;

        if (written != sizeof(pid_args) + 6) {
            Serial.printf("[PID] Failed to send defaults to servo %u\n", id);
            return false;
        }

        delay(SERVO_PID_WRITE_SETTLE_MS);
        if (!verify_servo_pid(id)) {
            return false;
        }
    }

    Serial.printf("[PID] Defaults verified: P=%u I=%u D=%u min_force=%u\n",
                  DEFAULT_SERVO_PID_P, DEFAULT_SERVO_PID_I, DEFAULT_SERVO_PID_D,
                  DEFAULT_SERVO_PID_MIN_FORCE);
    return true;
}

static bool write_default_servo_configuration() {
    return write_default_servo_pid();
}

static bool save_servo_pid_version(uint8_t version = SERVO_PID_CONFIG_VERSION) {
    if (!prefs.begin("robot_cfg", false)) {
        Serial.println("[PID] Failed to open NVS for writing");
        return false;
    }
    const size_t written = prefs.putUChar(SERVO_PID_VERSION_KEY, version);
    prefs.end();
    return written == sizeof(uint8_t);
}

static void initialize_servo_pid_once() {
    uint8_t stored_version = 0;
    if (prefs.begin("robot_cfg", true)) {
        stored_version = prefs.getUChar(SERVO_PID_VERSION_KEY, 0);
        prefs.end();
    }

    if (stored_version >= SERVO_PID_CONFIG_VERSION) {
        Serial.printf("[PID] NVS version %u found, keeping current servo PID\n", stored_version);
        return;
    }

    Serial.println("[PID] First boot detected, applying servo PID defaults");
    if (!write_default_servo_configuration()) {
        Serial.println("[PID] Defaults incomplete; NVS marker not written");
        return;
    }

    if (save_servo_pid_version()) {
        Serial.printf("[PID] NVS marker saved, version=%u\n", SERVO_PID_CONFIG_VERSION);
    } else {
        Serial.println("[PID] Failed to save NVS marker; defaults will retry next boot");
    }
}

void save_config() {
    prefs.begin("robot_cfg", false);
    prefs.putUChar("channel", global_channel);
    prefs.putUChar("acc", global_acc);

    prefs.putUChar("bt_mode", (uint8_t)current_bt_mode);
    prefs.putString("ps3_mac", String(PS3_MAC));

    prefs.putUChar("chs_type", chassis_type);
    prefs.putFloat("chs_wdia", chassis_wheel_diameter);
    prefs.putFloat("chs_wbase", chassis_wheel_base);
    prefs.putFloat("chs_twidth", chassis_track_width);
    prefs.putBytes("chs_mdir", chassis_motor_dir, 4);
    prefs.putChar("chs_maxspd", chassis_max_speed);

    prefs.putBytes("arm_kin", arm_kin, sizeof(arm_kin));
    prefs.end();
}

void load_config() {
    prefs.begin("robot_cfg", true);
    global_channel = prefs.getUChar("channel", 2);
    global_acc = prefs.getUChar("acc", 245);
    current_bt_mode = (BluetoothMode)prefs.getUChar("bt_mode", BT_MODE_BLE);

    String saved_mac = prefs.getString("ps3_mac", "10:00:00:00:85:95");
    strncpy(ps3_mac_buf, saved_mac.c_str(), sizeof(ps3_mac_buf) - 1);
    ps3_mac_buf[sizeof(ps3_mac_buf) - 1] = '\0';
    PS3_MAC = ps3_mac_buf;
    Serial.printf("[Config] PS3 MAC loaded: %s\n", PS3_MAC);

    float ag_x = prefs.getFloat("ag_x", 50.0f);
    float ag_y = prefs.getFloat("ag_y", 15.0f);
    float ag_z = prefs.getFloat("ag_z", 60.0f);

    aprilTagGrabber.setOffsets(ag_x, ag_y, ag_z);
    garbageGrabber.setOffsets(ag_x, ag_y, ag_z);
    colorGrabber.setOffsets(ag_x, ag_y, ag_z);
    calibrationGrabber.setOffsets(ag_x, ag_y, ag_z);

    chassis_type = prefs.getUChar("chs_type", CHASSIS_NONE);
    chassis_wheel_diameter = prefs.getFloat("chs_wdia", 25.0f);
    chassis_wheel_base = prefs.getFloat("chs_wbase", 190.0f);
    chassis_track_width = prefs.getFloat("chs_twidth", 95.0f);
    prefs.getBytes("chs_mdir", chassis_motor_dir, 4);
    chassis_max_speed = prefs.getChar("chs_maxspd", 100);
    Serial.printf("[Config] Chassis: type=%d, wheel_d=%.1f, base=%.1f, track=%.1f, max_spd=%d\n",
                  chassis_type, chassis_wheel_diameter, chassis_wheel_base, chassis_track_width, chassis_max_speed);
    apply_chassis_config_to_runtime();

    if (prefs.isKey("arm_kin")) {
        float saved_kin[9];
        prefs.getBytes("arm_kin", saved_kin, sizeof(saved_kin));
        if (fabsf(saved_kin[0] - arm_kin[0]) < 0.01f) {
            memcpy(arm_kin, saved_kin, sizeof(arm_kin));
        } else {
            Serial.println("[Config] Kinematics defaults changed, using new values");
        }
    }
    Serial.printf("[Config] Kinematics: L1=%.1f L2=%.1f base_high=%.1f\n",
                  arm_kin[0], arm_kin[1], arm_kin[8]);

    prefs.end();
}

void sys_timer_post_callback(TimerHandle_t xTimer)
{
    if(is_downloading) return;

    static uint32_t count= 0;
    if(count % BUZZER_UPDATE_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_BUZZER_UPDATE, NULL, 0, 0);
    }
    if(count % BAT_UPDATE_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_BAT_UPDATE, NULL, 0, 0);
    }

    if(count % 2000 == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_OLED_REFRESH, NULL, 0, 0);
    }

    count += TIMER_PERIOD;
}

static void sys_timer_sub_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if(is_downloading) return;

    switch(id) {

        case TIMING_EVENT_OLED_REFRESH:
            if(current_oled_mode == SHOW_STATUS) {
                if(millis() - last_display_time > 80) {

                    arm.board.oled.show_status(arm.current_pose.x, arm.current_pose.y, arm.current_pose.z,
                                     arm.current_pose.pitch, arm.current_pose.roll, arm.current_pose.claw);
                    last_display_time = millis();
                }
            } else if(current_oled_mode == SHOW_NET_INFO) {
                if (current_system_mode == MODE_ESPNOW_RUNNING) {
                     arm.board.oled.show_espnow_info(global_channel, global_acc, espnow_ctrl.is_unicast_mode, espnow_ctrl.getMacAddress());
                } else {
                     arm.board.oled.show_wifi_ap_info(String(AP_SSID), WiFi.softAPIP());
                }
            } else if (current_oled_mode == SHOW_CUSTOM) {
                arm.board.oled.show_custom();

                if(ps3_prompt_active && (millis() - ps3_prompt_start_time > 5000)) {
                    ps3_prompt_active = false;
                    current_oled_mode = SHOW_STATUS;
                }
            } else if (current_oled_mode == SHOW_ICON) {
            arm.board.oled.show_icon();
            }
            break;

        case TIMING_EVENT_BAT_UPDATE:
            arm.board.bat.update();
            break;

        case TIMING_EVENT_BUZZER_UPDATE:
            arm.board.buzzer.update();
            break;
    }
}

static void boot_button_sub_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if(is_downloading) return;

    arm.board.oled.set_icon(4);
    current_oled_mode = SHOW_ICON;
    noTone(23);

    uint32_t anim_start = millis();
    while (millis() - anim_start < 600) {
        arm.board.oled.show_icon();
        delay(30);
    }

    if (current_system_mode == MODE_WIFI_AP_RUNNING) {
        espnow_ctrl.stop();
        delay(50);

        if (global_channel < 1 || global_channel > 13) {
            global_channel = 2;
        }

        while(servo.uart->available()) servo.uart->read();

        if (!espnow_ctrl.begin(global_channel)) {
            current_system_mode = MODE_WIFI_AP_RUNNING;
            Serial.println("[ESP-NOW] start failed from boot button");
            current_oled_mode = SHOW_NET_INFO;
            return;
        }

        espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);

        uint8_t t_data[2];
        t_data[0] = 40;
        t_data[1] = 1;
        servo.tx_frame_write(0xFE, 3, t_data, 2);
        delay(10);

        uint8_t acc_data[2];
        acc_data[0] = 41;
        acc_data[1] = global_acc;
        servo.tx_frame_write(BROADCAST_ID, 3, acc_data, 2);
        delay(10);

        while(servo.uart->available()) servo.uart->read();

        espnow_ctrl.enable_sync(true);
        current_system_mode = MODE_ESPNOW_RUNNING;
    }
    else {
        espnow_ctrl.stop();
        delay(50);
        espnow_ctrl.startAP(AP_SSID, AP_PASS);
        espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);
        current_system_mode = MODE_WIFI_AP_RUNNING;
    }

    current_oled_mode = SHOW_NET_INFO;
}

enum Ps3ArmControlMode {
    PS3_ARM_MODE_COORDINATE = 0,
    PS3_ARM_MODE_SINGLE_SERVO = 1,
};
static Ps3ArmControlMode ps3_arm_control_mode = PS3_ARM_MODE_COORDINATE;
static int8_t ps3_speed_level = 50;
static uint32_t ps3_held_buttons = 0;
static uint32_t ps3_repeat_last_time = 0;
static const uint32_t PS3_REPEAT_INTERVAL = 80;
static const uint32_t PS3_SERVO_REPEAT_INTERVAL = 60;
static uint32_t ps3_last_combo_toggle_time = 0;
static const uint32_t PS3_COMBO_DEBOUNCE_MS = 800;
static const int16_t PS3_SERVO_STEP = 20;
static const uint8_t PS3_SERVO_ACC = 150;
static const int16_t PS3_SERVO_SPEED = 1500;

static float PS3_X_MIN = 1.0f, PS3_X_MAX = 400.0f;
static float PS3_Y_MIN = -450.0f, PS3_Y_MAX = 450.0f;
static float PS3_Z_MIN = 30.0f,  PS3_Z_MAX = 500.0f;
static float PS3_PITCH_MIN = -98.0f, PS3_PITCH_MAX = 12.0f;
static float PS3_ROLL_MIN = -60.0f,   PS3_ROLL_MAX = 60.0f;
static float PS3_CLAW_MIN = -30.0f,   PS3_CLAW_MAX = 50.0f;

static float ps3_clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static float ps3_target_x = 200.0f;
static float ps3_target_y = 0.0f;
static float ps3_target_z = 200.0f;
static float ps3_target_pitch = 0.0f;
static float ps3_target_roll = 0.0f;
static float ps3_target_claw = 0.0f;

static int16_t ps3_servo_pos[6] = {2048, 2087, 1198, 1238, 2048, 2048};

static const int16_t PS3_SERVO_MIN[6] = { 210, 200, 968,  728,  908, 1118};
static const int16_t PS3_SERVO_MAX[6] = {3826, 3800, 3200, 3218, 3128, 2828};

static void ps3_single_servo_move(uint8_t id, int16_t delta) {
    int16_t cur = ps3_servo_pos[id - 1];
    int16_t lo = PS3_SERVO_MIN[id - 1];
    int16_t hi = PS3_SERVO_MAX[id - 1];
    int16_t target = (int16_t)constrain((int32_t)cur + delta, lo, hi);
    servo.write_pos_ex(id, PS3_SERVO_ACC, PS3_SERVO_SPEED, target);
    ps3_servo_pos[id - 1] = target;
    arm.update_status();
    Serial.printf("[PS3 Servo] id=%u cur=%d target=%d delta=%d\n", id, cur, target, delta);
}

static const int16_t PS3_SERVO_HOME[6] = {2048, 2087, 1198, 1238, 2048, 2048};

static void ps3_reset_to_home() {
    arm.update_status();
    delay(50);

    float home_x, home_z;
    if (chassis_type == CHASSIS_MECANUM) {

        home_x = 278.0f;
        home_z = 180.0f;
    } else {

        home_x = 200.0f;
        home_z = 200.0f;
    }

    arm.move(home_x, 0, home_z, 0, 0, 0, 1500);
    arm.board.buzzer.set(100, 100, 1, 2000);
    ps3_target_x = home_x;
    ps3_target_y = 0.0f;
    ps3_target_z = home_z;
    ps3_target_pitch = 0.0f;
    ps3_target_roll = 0.0f;
    ps3_target_claw = 0.0f;

    for (int i = 0; i < 6; i++) ps3_servo_pos[i] = PS3_SERVO_HOME[i];
}

static void ps3_single_servo_repeat() {

    if (ps3_held_buttons & (1UL << 14)) ps3_single_servo_move(1, -PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 15)) ps3_single_servo_move(1,  PS3_SERVO_STEP);

    if (ps3_held_buttons & (1UL << 12)) ps3_single_servo_move(2,  -PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 13)) ps3_single_servo_move(2, PS3_SERVO_STEP);

    if (ps3_held_buttons & (1UL << 3))  ps3_single_servo_move(3,  PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 0))  ps3_single_servo_move(3, -PS3_SERVO_STEP);

    if (ps3_held_buttons & (1UL << 2))  ps3_single_servo_move(4, -PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 1))  ps3_single_servo_move(4,  PS3_SERVO_STEP);

    if (ps3_held_buttons & (1UL << 4))  ps3_single_servo_move(5,  PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 5))  ps3_single_servo_move(5, -PS3_SERVO_STEP);

    if (ps3_held_buttons & (1UL << 6))  ps3_single_servo_move(6, -PS3_SERVO_STEP);
    if (ps3_held_buttons & (1UL << 7))  ps3_single_servo_move(6,  PS3_SERVO_STEP);
}

static void ps3_coordinate_repeat() {
    float dx = 0, dy = 0, dz = 0, dp = 0, dr = 0, dc = 0;

    if (ps3_held_buttons & (1UL << 0))  dz -= 15;
    if (ps3_held_buttons & (1UL << 3))  dz += 15;
    if (ps3_held_buttons & (1UL << 2))  dp -= 5;
    if (ps3_held_buttons & (1UL << 1))  dp += 5;

    if (ps3_held_buttons & (1UL << 4))  dr += 5;
    if (ps3_held_buttons & (1UL << 5))  dr -= 5;

    if (ps3_held_buttons & (1UL << 6))  dc -= 5;
    if (ps3_held_buttons & (1UL << 7))  dc += 5;

    if (ps3_held_buttons & (1UL << 12)) dx += 15;
    if (ps3_held_buttons & (1UL << 13)) dx -= 15;
    if (ps3_held_buttons & (1UL << 14)) dy += 15;
    if (ps3_held_buttons & (1UL << 15)) dy -= 15;

    if (dx == 0 && dy == 0 && dz == 0 && dp == 0 && dr == 0 && dc == 0) return;

    ps3_target_x     = ps3_clamp(ps3_target_x + dx, PS3_X_MIN, PS3_X_MAX);
    ps3_target_y     = ps3_clamp(ps3_target_y + dy, PS3_Y_MIN, PS3_Y_MAX);
    ps3_target_z     = ps3_clamp(ps3_target_z + dz, PS3_Z_MIN, PS3_Z_MAX);
    ps3_target_pitch = ps3_clamp(ps3_target_pitch + dp, PS3_PITCH_MIN, PS3_PITCH_MAX);
    ps3_target_roll  = ps3_clamp(ps3_target_roll + dr, PS3_ROLL_MIN, PS3_ROLL_MAX);
    ps3_target_claw  = ps3_clamp(ps3_target_claw + dc, PS3_CLAW_MIN, PS3_CLAW_MAX);

    arm.move(ps3_target_x, ps3_target_y, ps3_target_z,
             ps3_target_pitch, ps3_target_roll, ps3_target_claw, 80);
}

void ps3_repeat_handler() {
    if (ps3_held_buttons == 0) return;
    uint32_t now = millis();

    if (ps3_arm_control_mode == PS3_ARM_MODE_SINGLE_SERVO) {
        if (now - ps3_repeat_last_time < PS3_SERVO_REPEAT_INTERVAL) return;
        ps3_repeat_last_time = now;
        ps3_single_servo_repeat();
    } else {
        if (now - ps3_repeat_last_time < PS3_REPEAT_INTERVAL) return;
        ps3_repeat_last_time = now;
        ps3_coordinate_repeat();
    }
}

void ps3_button_callback(int button, bool pressed) {

    if (pressed) {
        ps3_held_buttons |= (1UL << button);
    } else {
        ps3_held_buttons &= ~(1UL << button);
    }

    if(!pressed) return;

    if ((button == 8 || button == 9) &&
        (ps3_held_buttons & (1UL << 8)) && (ps3_held_buttons & (1UL << 9))) {
        uint32_t now = millis();

        if (now - ps3_last_combo_toggle_time > PS3_COMBO_DEBOUNCE_MS) {
            ps3_last_combo_toggle_time = now;
            ps3_arm_control_mode = (ps3_arm_control_mode == PS3_ARM_MODE_COORDINATE)
                ? PS3_ARM_MODE_SINGLE_SERVO : PS3_ARM_MODE_COORDINATE;

            arm.update_status();
            delay(50);
            pump_at32_feedback();
            ps3_target_x = arm.current_pose.x;
            ps3_target_y = arm.current_pose.y;
            ps3_target_z = arm.current_pose.z;
            ps3_target_pitch = arm.current_pose.pitch;
            ps3_target_roll = arm.current_pose.roll;
            ps3_target_claw = arm.current_pose.claw;

            if (ps3_arm_control_mode == PS3_ARM_MODE_SINGLE_SERVO) {
                int16_t spos[6];
                if (get_last_servo_positions(spos)) {
                    memcpy(ps3_servo_pos, spos, sizeof(ps3_servo_pos));
                }
            }

            arm.board.buzzer.set(80, 80, ps3_arm_control_mode == PS3_ARM_MODE_COORDINATE ? 1 : 2, 2000);
            Serial.printf("PS3 mode: %s x=%.1f y=%.1f z=%.1f\n",
                          ps3_arm_control_mode == PS3_ARM_MODE_COORDINATE ? "coordinate" : "single-servo",
                          ps3_target_x, ps3_target_y, ps3_target_z);
        }
        return;
    }

    if (button == 16) {
        arm.board.motor.stop();
        chassis_type = (chassis_type == CHASSIS_TANK) ? CHASSIS_MECANUM : CHASSIS_TANK;
        save_config();
        arm.board.buzzer.set(100, 100, chassis_type == CHASSIS_TANK ? 2 : 1, 2000);
        Serial.printf("[PS3] Chassis: %s\n", chassis_type == CHASSIS_TANK ? "tank" : "mecanum");
        return;
    }

    if (ps3_arm_control_mode == PS3_ARM_MODE_SINGLE_SERVO) {
        if (button == 9) {
            ps3_reset_to_home();
        }
        return;
    }

    switch(button) {
        case 8:
            if(current_oled_mode == SHOW_STATUS) {
                current_oled_mode = SHOW_NET_INFO;
            } else {
                current_oled_mode = SHOW_STATUS;
            }
            break;
        case 9:
            ps3_reset_to_home();
            break;
        case 10:
            ps3_speed_level = constrain(ps3_speed_level + 10, 10, 100);
            arm.board.buzzer.set(100, 100, 1, 2000);
            Serial.printf("[PS3] Speed: %d\n", ps3_speed_level);
            break;
        case 11:
            ps3_speed_level = constrain(ps3_speed_level - 10, 10, 100);
            arm.board.buzzer.set(100, 100, 1, 1500);
            Serial.printf("[PS3] Speed: %d\n", ps3_speed_level);
            break;
    }
}

static int8_t ps3_left_x = 0, ps3_left_y = 0;
static int8_t ps3_right_x = 0, ps3_right_y = 0;

void ps3_analog_callback(int stick, int8_t x, int8_t y) {
    if(abs(x) < 15) x = 0;
    if(abs(y) < 15) y = 0;

    if(stick == 0) { ps3_left_x = x; ps3_left_y = y; }
    else if(stick == 1) { ps3_right_x = x; ps3_right_y = y; }

    int8_t vy = (int8_t)constrain(-ps3_left_x * ps3_speed_level / 128, -100, 100);
    int8_t vz = (int8_t)constrain(-ps3_right_x * ps3_speed_level / 128, -100, 100);
    int8_t vx = (int8_t)constrain(-ps3_left_y * ps3_speed_level / 128, -100, 100);

    if (chassis_type == CHASSIS_TANK) vy = 0;
    arm.board.mecanum.set_velocity(vx, vy, vz);
}

static uint8_t user_button_press_count = 0;
static uint32_t user_button_last_press = 0;

#define ACTION_RECORD_FILE          "/action_record.bin"
#define ACTION_RECORD_INTERVAL      10
#define ACTION_RECORD_MAX_DURATION_MS 60000
#define ACTION_PLAYBACK_INTERVAL    30
#define ACTION_PLAYBACK_PREROLL_MS  0
#define ACTION_PLAYBACK_SERVICE_MS  5
#define ACTION_JSON_DEBUG_INTERVAL  100
#define ACTION_EDIT_SERVO_ACC       200
#define ACTION_NORMAL_SERVO_ACC     254
#define ACTION_PLAYBACK_SPEED       2000
#define ACTION_JSON_DEBUG_ENABLE    0
#define ACTION_MAX_FRAMES           ((ACTION_RECORD_MAX_DURATION_MS / ACTION_RECORD_INTERVAL) + 2)

struct ActionFrame {
    int16_t servo_pos[6];
    uint32_t timestamp;
};

static bool action_edit_mode = false;
static bool action_recording = false;
static uint16_t recorded_frame_count = 0;
static uint32_t record_start_time = 0;
static int16_t last_recorded_pos[6] = {2048, 2048, 2048, 2048, 2048, 2048};
static bool action_playback_active = false;
static uint16_t playback_index = 0;
static bool action_servo_stream_active = false;
static volatile bool action_auto_stop_pending = false;
static volatile bool action_record_overflow = false;
static uint16_t action_record_capacity = 0;
static volatile uint16_t action_record_dropped_frames = 0;
static uint16_t action_file_count = 0;
static uint32_t action_last_record_timestamp = 0;
static uint32_t action_last_json_ms = 0;
static volatile uint32_t action_blocked_cmd_count = 0;
static File action_record_file;
static bool action_record_file_open = false;

enum ActionEditRequest : uint32_t {
    ACTION_REQ_ENTER     = 1UL << 0,
    ACTION_REQ_EXIT      = 1UL << 1,
    ACTION_REQ_START     = 1UL << 2,
    ACTION_REQ_STOP      = 1UL << 3,
    ACTION_REQ_PLAY      = 1UL << 4,
    ACTION_REQ_STOP_PLAY = 1UL << 5,
    ACTION_REQ_CLEAR     = 1UL << 6,
};

static const uint32_t ACTION_REQ_ALL =
    ACTION_REQ_ENTER |
    ACTION_REQ_EXIT |
    ACTION_REQ_START |
    ACTION_REQ_STOP |
    ACTION_REQ_PLAY |
    ACTION_REQ_STOP_PLAY |
    ACTION_REQ_CLEAR;

static portMUX_TYPE action_request_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE action_record_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t action_request_flags = 0;

static void action_queue_request(uint32_t flags) {
    portENTER_CRITICAL(&action_request_mux);
    action_request_flags |= flags;
    portEXIT_CRITICAL(&action_request_mux);
}

static uint32_t action_take_requests(uint32_t mask) {
    portENTER_CRITICAL(&action_request_mux);
    uint32_t flags = action_request_flags & mask;
    action_request_flags &= ~flags;
    portEXIT_CRITICAL(&action_request_mux);
    return flags;
}

static bool action_has_request(uint32_t mask) {
    portENTER_CRITICAL(&action_request_mux);
    bool has = (action_request_flags & mask) != 0;
    portEXIT_CRITICAL(&action_request_mux);
    return has;
}

static bool action_edit_active_or_pending() {
    return action_edit_mode ||
           action_recording ||
           action_playback_active ||
           action_servo_stream_active ||
           action_auto_stop_pending ||
           action_has_request(ACTION_REQ_ALL);
}

static bool action_should_block_external_servo_command(const PacketTypeDef* pkt) {
    if (!action_edit_active_or_pending()) {
        return false;
    }

    if (pkt->elements.id != 0xFF) {
        return (pkt->elements.id >= 1 && pkt->elements.id <= 6) ||
               pkt->elements.id == 0xFE;
    }

    switch (pkt->elements.cmd) {
        case CMD_ACTION_GROUP_RUN:
        case CMD_ACTION_GROUP_STOP:
        case CMD_ACTION_GROUP_ERASE:
        case CMD_FKINE_RESULT_GET:
        case CMD_IKINE_RESULT_GET:
        case CMD_COORDINATE_SET:
        case CMD_GET_CUR_COORDS:
        case CMD_SET_POS_OFFSET:
        case CMD_GET_POS_OFFSET:
        case CMD_SET_PID_PARAM:
        case CMD_GET_PID_PARAM:
        case CMD_SET_GLOBAL_ACC:
        case CMD_ESPNOW_SYNC_CTRL:
        case CMD_ARM_MOVE_INC:
        case CMD_MOVE_INC:
        case CMD_ARM_SERVO_SINGLE:
        case CMD_ARM_RESET:
        case CMD_SET_MOVE_ACC:
        case CMD_READ_ALL_SERVOS:
        case CMD_SET_TORQUE:
        case CMD_PC_SYNC_TEACH:
        case CMD_GET_REAL_JOINT_ANGLES:
        case CMD_GET_REAL_TCP_POSE:
        case CMD_SYNC_WRITE_SERVOS:
        case CMD_SERVO_READ_OVERLOAD:
        case CMD_SERVO_WRITE_OVERLOAD:
        case CMD_SERVO_READ_BAUD:
        case CMD_SERVO_WRITE_BAUD:
        case CMD_SERVO_READ_MAX_TORQUE:
        case CMD_SERVO_WRITE_MAX_TORQUE:
        case CMD_SERVO_READ_ANGLE_LIMIT:
        case CMD_SERVO_WRITE_ANGLE_LIMIT:
        case CMD_SERVO_CALI_POS:
        case CMD_SET_COORD_LIMITS:
        case CMD_GET_COORD_LIMITS:
            return true;
        default:
            return false;
    }
}

static void action_clear_requests(uint32_t mask) {
    (void)action_take_requests(mask);
}

static void action_frames_clear() {
    portENTER_CRITICAL(&action_record_mux);
    recorded_frame_count = 0;
    action_last_record_timestamp = 0;
    action_auto_stop_pending = false;
    action_record_overflow = false;
    action_record_dropped_frames = 0;
    portEXIT_CRITICAL(&action_record_mux);
    action_file_count = 0;
}

static void action_free_record_buffer() {
    if (action_record_file_open) {
        action_record_file.flush();
        action_record_file.close();
        action_record_file_open = false;
    }
    action_record_capacity = 0;
}

static bool action_alloc_record_buffer() {
    action_free_record_buffer();

    action_record_file = LittleFS.open(ACTION_RECORD_FILE, "wb");
    if (!action_record_file) {
        Serial.println("[ActionEdit] Failed to open record file for streaming");
        return false;
    }

    uint16_t count = 0;
    if (action_record_file.write((uint8_t*)&count, sizeof(count)) != sizeof(count)) {
        action_record_file.close();
        action_record_file_open = false;
        Serial.println("[ActionEdit] Failed to write record header");
        return false;
    }

    action_record_file_open = true;
    action_record_capacity = ACTION_MAX_FRAMES;

    Serial.printf("[ActionEdit] Record file streaming: max_frames=%u\n", action_record_capacity);
    return true;
}

static void stop_recording_and_save();

static void action_append_record_frame(const int16_t positions[6], uint32_t timestamp) {
    if (!action_recording) {
        return;
    }

    portENTER_CRITICAL(&action_record_mux);
    if (!action_recording) {
        portEXIT_CRITICAL(&action_record_mux);
        return;
    }

    if (!action_record_file_open || action_record_capacity == 0 || recorded_frame_count >= action_record_capacity) {
        action_recording = false;
        action_auto_stop_pending = true;
        action_record_overflow = true;
        action_record_dropped_frames++;
        portEXIT_CRITICAL(&action_record_mux);
        return;
    }

    if (recorded_frame_count > 0 && timestamp <= action_last_record_timestamp) {
        timestamp = action_last_record_timestamp + 1;
    }

    if (recorded_frame_count > 0 && (uint32_t)(timestamp - action_last_record_timestamp) < ACTION_RECORD_INTERVAL) {
        portEXIT_CRITICAL(&action_record_mux);
        return;
    }

    uint16_t frame_index = recorded_frame_count;
    uint32_t saved_timestamp = (uint32_t)frame_index * ACTION_RECORD_INTERVAL;
    recorded_frame_count++;
    action_last_record_timestamp = timestamp;
    portEXIT_CRITICAL(&action_record_mux);

    if (action_record_file.write((uint8_t*)positions, 6 * sizeof(int16_t)) != 6 * sizeof(int16_t) ||
        action_record_file.write((uint8_t*)&saved_timestamp, sizeof(saved_timestamp)) != sizeof(saved_timestamp)) {
        portENTER_CRITICAL(&action_record_mux);
        action_recording = false;
        action_auto_stop_pending = true;
        action_record_overflow = true;
        action_record_dropped_frames++;
        portEXIT_CRITICAL(&action_record_mux);
        return;
    }

    memcpy(last_recorded_pos, positions, sizeof(last_recorded_pos));
}

static void action_record_feedback_frame(const int16_t positions[6]) {
    if (!action_recording) {
        return;
    }

    uint32_t now = millis();
    uint32_t timestamp = now - record_start_time;

    if (timestamp > ACTION_RECORD_MAX_DURATION_MS) {
        portENTER_CRITICAL(&action_record_mux);
        action_recording = false;
        action_auto_stop_pending = true;
        portEXIT_CRITICAL(&action_record_mux);
        return;
    }

    action_append_record_frame(positions, timestamp);

    if (timestamp >= ACTION_RECORD_MAX_DURATION_MS) {
        portENTER_CRITICAL(&action_record_mux);
        action_recording = false;
        action_auto_stop_pending = true;
        portEXIT_CRITICAL(&action_record_mux);
    }
}

static void action_write_servo_positions(const int16_t src_positions[6], int16_t speed) {
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    int16_t positions[6];

    memcpy(positions, src_positions, sizeof(positions));

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;
    servo.sync_write_pos_speed(ids, positions, speed, 6);
    servo_uart_busy = false;
}

static void write_servo_acc(uint8_t acc) {
    uint8_t acc_data[2] = {41, acc};

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;
    servo.tx_frame_write(0xFE, 3, acc_data, 2);
    delay(5);
    servo_uart_busy = false;
}

static void set_servo_realtime_mode(bool enable, uint8_t acc = global_acc) {
    uint8_t t_data[2] = {40, (uint8_t)(enable ? 1 : 0)};

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;
    for (int i = 0; i < (enable ? 3 : 1); i++) {
        servo.tx_frame_write(0xFE, 3, t_data, 2);
        delay(5);
    }

    if (enable) {
        uint8_t acc_data[2] = {41, acc};
        servo.tx_frame_write(0xFE, 3, acc_data, 2);
        delay(5);
    }
    servo_uart_busy = false;
}

static void action_set_servo_stream(bool enable) {
    if (action_servo_stream_active == enable) {
        return;
    }

    uint8_t data[1] = { (uint8_t)(enable ? 1 : 0) };

    if (enable) {
        g_last_servo_stream_ms = 0;
    }

    while (servo_uart_busy) { delayMicroseconds(100); }
    servo_uart_busy = true;
    servo.tx_frame_write(0xFF, CMD_SERVO_STREAM, data, 1);
    servo_uart_busy = false;

    action_servo_stream_active = enable;
}

static void action_print_json(const char* state, const int16_t positions[6], uint16_t index, uint16_t count) {
#if !ACTION_JSON_DEBUG_ENABLE
    (void)state;
    (void)positions;
    (void)index;
    (void)count;
    return;
#endif

    uint32_t now = millis();
    if ((uint32_t)(now - action_last_json_ms) < ACTION_JSON_DEBUG_INTERVAL) {
        return;
    }
    action_last_json_ms = now;

    const int16_t* pos = positions ? positions : g_last_servo_positions;

    Serial.printf("{\"type\":\"action_edit\",\"state\":\"%s\",\"t\":%lu,\"frames\":%u,\"idx\":%u,\"count\":%u,\"stream_age\":",
                  state,
                  (unsigned long)now,
                  recorded_frame_count,
                  index,
                  count);
    if (g_last_servo_stream_ms == 0) {
        Serial.print("null");
    } else {
        Serial.print((unsigned long)(now - g_last_servo_stream_ms));
    }
    Serial.printf(",\"blocked\":%lu,\"pos\":[%d,%d,%d,%d,%d,%d]}\n",
                  (unsigned long)action_blocked_cmd_count,
                  pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
}

static bool action_save_recorded_frames_to_file() {
    if (!action_record_file_open) {
        Serial.println("[ActionEdit] No record file to save");
        return false;
    }

    uint16_t count = recorded_frame_count;
    action_record_file.flush();
    if (!action_record_file.seek(0)) {
        action_record_file.close();
        action_record_file_open = false;
        Serial.println("[ActionEdit] Failed to seek record header");
        return false;
    }

    if (action_record_file.write((uint8_t*)&count, sizeof(uint16_t)) != sizeof(uint16_t)) {
        action_record_file.close();
        action_record_file_open = false;
        Serial.println("[ActionEdit] Failed to write record header");
        return false;
    }

    action_record_file.flush();
    action_record_file.close();
    action_record_file_open = false;
    action_file_count = count;
    Serial.printf("[ActionEdit] Saved %u frames to Flash\n", action_file_count);
    return true;
}

static bool action_read_frame(File& file, ActionFrame& frame) {
    if (file.read((uint8_t*)frame.servo_pos, 6 * sizeof(int16_t)) != 6 * sizeof(int16_t)) {
        return false;
    }
    return file.read((uint8_t*)&frame.timestamp, sizeof(uint32_t)) == sizeof(uint32_t);
}

static void action_beep(int count) {
    arm.board.buzzer.set(100, 100, count, 2000);
}

static void action_playback_wait_until(uint32_t due_time) {
    while (action_playback_active && (int32_t)(due_time - millis()) > 0) {
        serial_port.rec_handler();
        if (current_system_mode == MODE_WIFI_AP_RUNNING) {
            espnow_ctrl.server_loop();
        }
        pump_at32_feedback();

        uint32_t stop_flags = action_take_requests(ACTION_REQ_STOP_PLAY | ACTION_REQ_EXIT);
        if (stop_flags & ACTION_REQ_STOP_PLAY) {
            action_playback_active = false;
            break;
        }
        if (stop_flags & ACTION_REQ_EXIT) {
            action_queue_request(ACTION_REQ_EXIT);
            action_playback_active = false;
            break;
        }

        int32_t remaining = (int32_t)(due_time - millis());
        if (remaining > 2) {
            vTaskDelay(pdMS_TO_TICKS(ACTION_PLAYBACK_SERVICE_MS));
        } else if (remaining > 0) {
            delayMicroseconds(200);
        }
    }
}

static void action_playback_wait(uint32_t wait_ms) {
    action_playback_wait_until(millis() + wait_ms);
}

static void enter_action_edit_mode() {
    action_edit_mode = true;
    action_recording = false;
    action_playback_active = false;
    action_set_servo_stream(false);
    action_frames_clear();
    action_free_record_buffer();
    playback_index = 0;
    action_last_json_ms = 0;
    action_blocked_cmd_count = 0;

    action_beep(1);
    write_servo_acc(ACTION_EDIT_SERVO_ACC);
    arm.set_torque(false);

    arm.board.oled.set_custom_text(0, "Action Edit");
    arm.board.oled.set_custom_text(1, "Mode");
    arm.board.oled.set_custom_text(2, "Torque: OFF");
    arm.board.oled.set_custom_text(3, "Ready");
    current_oled_mode = SHOW_CUSTOM;
    arm.board.oled.show_custom();

    Serial.println("[ActionEdit] Entered edit mode - torque off");
}

static void exit_action_edit_mode() {
    action_edit_mode = false;
    action_recording = false;
    action_playback_active = false;
    action_set_servo_stream(false);
    playback_index = 0;

    action_beep(1);
    set_servo_realtime_mode(false);
    write_servo_acc(ACTION_NORMAL_SERVO_ACC);
    arm.set_torque(true);

    current_oled_mode = SHOW_STATUS;
    arm.board.oled.show_status(
        arm.current_pose.x,
        arm.current_pose.y,
        arm.current_pose.z,
        arm.current_pose.pitch,
        arm.current_pose.roll,
        arm.current_pose.claw
    );

    Serial.println("[ActionEdit] Exited edit mode - torque on");
}

static void start_recording() {
    action_recording = false;
    action_playback_active = false;
    action_set_servo_stream(false);
    arm.set_torque(false);
    action_frames_clear();
    playback_index = 0;
    action_last_json_ms = 0;
    action_blocked_cmd_count = 0;

    if (!action_alloc_record_buffer()) {
        action_beep(2);
        return;
    }

    record_start_time = millis();

    action_recording = true;
    action_set_servo_stream(true);
    action_print_json("record_wait", last_recorded_pos, 0, 0);

    action_beep(2);

    arm.board.oled.set_custom_text(0, "Recording...");
    arm.board.oled.set_custom_text(1, "");
    arm.board.oled.set_custom_text(2, "");
    arm.board.oled.set_custom_text(3, "");
    arm.board.oled.show_custom();

    Serial.println("[ActionEdit] Recording started, waiting for 0x68 servo stream");
}

static void stop_recording_and_save() {
    action_recording = false;
    action_auto_stop_pending = false;
    action_set_servo_stream(false);

    bool saved = action_save_recorded_frames_to_file();
    if (!saved) {
        action_record_overflow = true;
    }
    action_beep(3);

    uint16_t count = action_file_count;
    arm.board.oled.set_custom_text(0, saved ? (action_record_overflow ? "Saved Partial" : "Saved!") : "Save Failed");
    arm.board.oled.set_custom_text(1, String("Frames: ") + count);
    arm.board.oled.set_custom_text(2, "");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();

    uint32_t duration = 0;
    if (count >= 2) {
        duration = action_last_record_timestamp;
    }
    Serial.printf("[ActionEdit] Recording stopped, saved %u frames, duration %lu ms, overflow=%d dropped=%u\n",
                  count, duration, action_record_overflow ? 1 : 0, action_record_dropped_frames);

    action_free_record_buffer();
}

static void start_playback() {
    action_set_servo_stream(false);

    File file = LittleFS.open(ACTION_RECORD_FILE, "rb");
    if (!file) {
        Serial.println("[ActionEdit] No action to play!");
        action_beep(2);
        return;
    }

    uint16_t count = 0;
    if (file.read((uint8_t*)&count, sizeof(uint16_t)) != sizeof(uint16_t) || count < 2) {
        file.close();
        Serial.println("[ActionEdit] Too few frames to play!");
        action_beep(2);
        return;
    }

    action_playback_active = true;
    action_beep(2);

    recorded_frame_count = count;
    arm.board.oled.set_custom_text(0, "Playing...");
    arm.board.oled.set_custom_text(1, String("Frames: ") + count);
    arm.board.oled.set_custom_text(2, "");
    arm.board.oled.set_custom_text(3, "");
    arm.board.oled.show_custom();

    Serial.printf("[ActionEdit] Playback started (%u frames)\n", count);

    arm.set_torque(true);
    set_servo_realtime_mode(true, ACTION_EDIT_SERVO_ACC);

    ActionFrame first_frame;
    if (!action_read_frame(file, first_frame)) {
        file.close();
        action_playback_active = false;
        set_servo_realtime_mode(false);
        arm.set_torque(true);
        action_beep(2);
        Serial.println("[ActionEdit] Failed to read first frame");
        return;
    }

    action_write_servo_positions(first_frame.servo_pos, ACTION_PLAYBACK_SPEED);
    action_print_json("play", first_frame.servo_pos, 0, count);
    action_playback_wait(ACTION_PLAYBACK_PREROLL_MS);

    bool playback_completed = true;
    uint32_t playback_start = millis();
    uint32_t first_timestamp = first_frame.timestamp;

    for (uint16_t frame_idx = 1; action_playback_active && frame_idx < count; frame_idx++) {
        ActionFrame frame;
        if (!action_read_frame(file, frame)) {
            playback_completed = false;
            break;
        }

        uint32_t frame_time = frame.timestamp - first_timestamp;
        uint32_t due_time = playback_start + frame_time;
        action_playback_wait_until(due_time);

        if (!action_playback_active) {
            playback_completed = false;
            break;
        }

        action_write_servo_positions(frame.servo_pos, ACTION_PLAYBACK_SPEED);
        playback_index = frame_idx;
        action_print_json("play", frame.servo_pos, frame_idx, count);
    }

    file.close();

    if (!action_playback_active) {
        playback_completed = false;
    } else {
        action_playback_wait(ACTION_PLAYBACK_INTERVAL);
    }

    action_playback_active = false;
    playback_index = 0;
    set_servo_realtime_mode(false);
    arm.set_torque(true);

    if (playback_completed) {
        arm.board.oled.set_custom_text(0, "Play Done!");
        arm.board.oled.set_custom_text(1, String("Frames: ") + count);
        arm.board.oled.set_custom_text(2, "Torque: ON");
        arm.board.oled.set_custom_text(3, "Ready");
        arm.board.oled.show_custom();

        Serial.println("[ActionEdit] Playback completed");
    }
}

static void stop_playback() {
    action_playback_active = false;
    playback_index = 0;
    action_beep(1);
    set_servo_realtime_mode(false);
    arm.set_torque(true);

    arm.board.oled.set_custom_text(0, "Action Edit");
    arm.board.oled.set_custom_text(1, "Mode");
    arm.board.oled.set_custom_text(2, "Torque: ON");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();

    Serial.println("[ActionEdit] Playback stopped");
}

static void clear_recorded_data() {
    action_recording = false;
    action_playback_active = false;
    action_set_servo_stream(false);
    action_frames_clear();
    action_free_record_buffer();
    record_start_time = 0;
    playback_index = 0;
    LittleFS.remove(ACTION_RECORD_FILE);
    action_beep(2);

    arm.board.oled.set_custom_text(0, "Cleared!");
    arm.board.oled.set_custom_text(1, "No action");
    arm.board.oled.set_custom_text(2, "");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();

    Serial.println("[ActionEdit] Action data cleared");
}

static void handle_action_recording() {
    if (action_auto_stop_pending) {
        stop_recording_and_save();
        return;
    }

    if (!action_recording) return;

    pump_at32_feedback();
    if (g_last_servo_stream_ms == 0) {
        action_print_json("record_wait", last_recorded_pos, recorded_frame_count, recorded_frame_count);
    }

    uint32_t now = millis();
    uint32_t elapsed = now - record_start_time;

    if (elapsed >= ACTION_RECORD_MAX_DURATION_MS) {
        stop_recording_and_save();
        return;
    }
}

static void handle_action_edit_requests() {
    uint32_t requests = action_take_requests(ACTION_REQ_ALL);

    if (!requests) {
        return;
    }

    if (requests & ACTION_REQ_STOP_PLAY) {
        if (action_playback_active) {
            stop_playback();
        }
    }

    if (requests & ACTION_REQ_STOP) {
        if (action_recording || action_auto_stop_pending) {
            stop_recording_and_save();
        }
    }

    if (requests & ACTION_REQ_CLEAR) {
        if (!action_recording && !action_playback_active) {
            clear_recorded_data();
        }
    }

    if (requests & ACTION_REQ_ENTER) {
        enter_action_edit_mode();
    }

    if (requests & ACTION_REQ_EXIT) {
        if (action_recording || action_auto_stop_pending) {
            stop_recording_and_save();
        }
        exit_action_edit_mode();
        return;
    }

    if ((requests & ACTION_REQ_START) && action_edit_mode && !action_recording && !action_playback_active) {
        start_recording();
    }

    if ((requests & ACTION_REQ_PLAY) && action_edit_mode && !action_recording && !action_playback_active) {
        start_playback();
    }
}

#define SYNC_TEACH_RECORD_FILE       "/sync_teach.bin"
#define SYNC_TEACH_MAX_FRAMES        65535
#define SYNC_TEACH_MAX_DURATION_MS   60000
#define SYNC_TEACH_RING_FRAMES       256
#define SYNC_TEACH_FLUSH_BATCH       4
#define SYNC_TEACH_PLAYBACK_SERVICE_MS 1

struct SyncTeachFrame {
    int16_t master_pos[6];
    uint32_t timestamp;
};

static volatile bool sync_teach_mode = false;
static volatile bool sync_teach_recording = false;
static volatile bool sync_teach_playback_active = false;
static volatile bool sync_teach_overflow = false;
static volatile bool sync_teach_auto_stop_pending = false;
static bool sync_teach_have_first_seq = false;
static uint32_t sync_teach_first_seq = 0;
static uint32_t sync_teach_first_arrival_ms = 0;
static uint32_t sync_teach_last_timestamp = 0;
static volatile uint16_t sync_teach_frame_count = 0;
static uint16_t sync_teach_playback_index = 0;
static SyncTeachFrame sync_teach_ring[SYNC_TEACH_RING_FRAMES];
static volatile uint16_t sync_teach_ring_head = 0;
static volatile uint16_t sync_teach_ring_tail = 0;
static volatile uint16_t sync_teach_ring_count = 0;
static volatile uint16_t sync_teach_dropped_frames = 0;
static File sync_teach_record_file;
static bool sync_teach_record_file_open = false;
static uint16_t sync_teach_file_count = 0;
static uint32_t sync_teach_last_file_flush_ms = 0;

enum SyncTeachRequest : uint32_t {
    SYNC_TEACH_REQ_ENTER     = 1UL << 0,
    SYNC_TEACH_REQ_EXIT      = 1UL << 1,
    SYNC_TEACH_REQ_START     = 1UL << 2,
    SYNC_TEACH_REQ_STOP      = 1UL << 3,
    SYNC_TEACH_REQ_PLAY      = 1UL << 4,
    SYNC_TEACH_REQ_STOP_PLAY = 1UL << 5,
    SYNC_TEACH_REQ_CLEAR     = 1UL << 6,
};

static portMUX_TYPE sync_teach_mux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE sync_teach_request_mux = portMUX_INITIALIZER_UNLOCKED;
static uint32_t sync_teach_request_flags = 0;

static void sync_teach_queue_request(uint32_t flags) {
    portENTER_CRITICAL(&sync_teach_request_mux);
    sync_teach_request_flags |= flags;
    portEXIT_CRITICAL(&sync_teach_request_mux);
}

static uint32_t sync_teach_take_requests(uint32_t mask) {
    portENTER_CRITICAL(&sync_teach_request_mux);
    uint32_t flags = sync_teach_request_flags & mask;
    sync_teach_request_flags &= ~flags;
    portEXIT_CRITICAL(&sync_teach_request_mux);
    return flags;
}

static bool sync_teach_has_request(uint32_t mask) {
    portENTER_CRITICAL(&sync_teach_request_mux);
    bool has = (sync_teach_request_flags & mask) != 0;
    portEXIT_CRITICAL(&sync_teach_request_mux);
    return has;
}

static void sync_teach_frames_clear() {
    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_frame_count = 0;
    sync_teach_last_timestamp = 0;
    sync_teach_have_first_seq = false;
    sync_teach_ring_head = 0;
    sync_teach_ring_tail = 0;
    sync_teach_ring_count = 0;
    sync_teach_dropped_frames = 0;
    portEXIT_CRITICAL(&sync_teach_mux);
}

static bool sync_teach_open_record_file() {
    if (sync_teach_record_file_open) {
        sync_teach_record_file.close();
        sync_teach_record_file_open = false;
    }

    sync_teach_record_file = LittleFS.open(SYNC_TEACH_RECORD_FILE, "wb");
    if (!sync_teach_record_file) {
        Serial.println("[SyncTeach] Failed to open record file");
        return false;
    }

    uint16_t zero = 0;
    sync_teach_record_file.write((uint8_t*)&zero, sizeof(uint16_t));
    sync_teach_file_count = 0;
    sync_teach_last_file_flush_ms = millis();
    sync_teach_record_file_open = true;
    return true;
}

static void sync_teach_flush_record_file(uint16_t max_frames) {
    if (!sync_teach_record_file_open) {
        return;
    }

    uint16_t flushed = 0;
    while (flushed < max_frames) {
        SyncTeachFrame frame;
        bool has_frame = false;

        portENTER_CRITICAL(&sync_teach_mux);
        if (sync_teach_ring_count > 0) {
            frame = sync_teach_ring[sync_teach_ring_tail];
            sync_teach_ring_tail = (uint16_t)((sync_teach_ring_tail + 1) % SYNC_TEACH_RING_FRAMES);
            sync_teach_ring_count--;
            has_frame = true;
        }
        portEXIT_CRITICAL(&sync_teach_mux);

        if (!has_frame) {
            break;
        }

        sync_teach_record_file.write((uint8_t*)frame.master_pos, 6 * sizeof(int16_t));
        sync_teach_record_file.write((uint8_t*)&frame.timestamp, sizeof(uint32_t));
        sync_teach_file_count++;
        flushed++;
    }

    uint32_t now = millis();
    if ((uint32_t)(now - sync_teach_last_file_flush_ms) >= 1000) {
        sync_teach_record_file.flush();
        sync_teach_last_file_flush_ms = now;
    }
}

static void sync_teach_flush_record_file_all() {
    while (true) {
        uint16_t pending = 0;
        portENTER_CRITICAL(&sync_teach_mux);
        pending = sync_teach_ring_count;
        portEXIT_CRITICAL(&sync_teach_mux);

        if (pending == 0) {
            return;
        }

        sync_teach_flush_record_file(pending);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

static void sync_teach_close_record_file() {
    if (!sync_teach_record_file_open) {
        return;
    }

    sync_teach_flush_record_file_all();
    sync_teach_record_file.seek(0, SeekSet);
    sync_teach_record_file.write((uint8_t*)&sync_teach_file_count, sizeof(uint16_t));
    sync_teach_record_file.flush();
    sync_teach_record_file.close();
    sync_teach_record_file_open = false;
    sync_teach_frame_count = sync_teach_file_count;
    Serial.printf("[SyncTeach] Saved %u frames to Flash\n", sync_teach_file_count);
}

static void sync_teach_abort_record_from_callback() {
    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_recording = false;
    sync_teach_overflow = true;
    sync_teach_auto_stop_pending = true;
    portEXIT_CRITICAL(&sync_teach_mux);
}

bool sync_teach_handle_master_packet(uint32_t seq, const int16_t master_positions[6]) {
    if (!sync_teach_mode && !sync_teach_recording && !sync_teach_playback_active) {
        return false;
    }

    if (sync_teach_playback_active) {
        return true;
    }

    if (!sync_teach_recording) {
        return true;
    }

    SyncTeachFrame frame;
    uint32_t now = millis();

    portENTER_CRITICAL(&sync_teach_mux);
    if (!sync_teach_recording) {
        portEXIT_CRITICAL(&sync_teach_mux);
        return true;
    }

    if (!sync_teach_have_first_seq) {
        sync_teach_have_first_seq = true;
        sync_teach_first_seq = seq;
        sync_teach_first_arrival_ms = now;
        frame.timestamp = 0;
    } else {
        frame.timestamp = seq - sync_teach_first_seq;
        if (frame.timestamp > 3600000UL) {
            frame.timestamp = now - sync_teach_first_arrival_ms;
        }
    }

    if (sync_teach_frame_count > 0 && frame.timestamp <= sync_teach_last_timestamp) {
        frame.timestamp = sync_teach_last_timestamp + 1;
    }
    if (frame.timestamp > SYNC_TEACH_MAX_DURATION_MS) {
        sync_teach_recording = false;
        sync_teach_auto_stop_pending = true;
        portEXIT_CRITICAL(&sync_teach_mux);
        return true;
    }
    portEXIT_CRITICAL(&sync_teach_mux);

    for (int i = 0; i < 6; i++) {
        frame.master_pos[i] = master_positions[i];
    }

    portENTER_CRITICAL(&sync_teach_mux);
    if (!sync_teach_recording) {
        portEXIT_CRITICAL(&sync_teach_mux);
        return true;
    }
    if (sync_teach_frame_count >= SYNC_TEACH_MAX_FRAMES || sync_teach_ring_count >= SYNC_TEACH_RING_FRAMES) {
        sync_teach_recording = false;
        sync_teach_overflow = true;
        sync_teach_auto_stop_pending = true;
        sync_teach_dropped_frames++;
        portEXIT_CRITICAL(&sync_teach_mux);
        return true;
    }

    sync_teach_ring[sync_teach_ring_head] = frame;
    sync_teach_ring_head = (uint16_t)((sync_teach_ring_head + 1) % SYNC_TEACH_RING_FRAMES);
    sync_teach_ring_count++;
    sync_teach_frame_count++;
    sync_teach_last_timestamp = frame.timestamp;
    portEXIT_CRITICAL(&sync_teach_mux);
    return false;
}

static void sync_teach_beep(int count) {
    arm.board.buzzer.set(100, 100, count, 2000);
}

static void sync_teach_set_realtime_servo_mode(bool enable) {
    set_servo_realtime_mode(enable);
}

static bool sync_teach_ensure_espnow_running() {
    if (current_system_mode == MODE_ESPNOW_RUNNING && espnow_ctrl.is_espnow_running) {
        espnow_ctrl.enable_sync(false);
        return true;
    }

    espnow_ctrl.stop();
    delay(50);

    if (global_channel < 1 || global_channel > 13) {
        global_channel = 2;
    }

    while (servo.uart->available()) {
        servo.uart->read();
    }

    if (!espnow_ctrl.begin(global_channel)) {
        current_system_mode = MODE_WIFI_AP_RUNNING;
        espnow_ctrl.startAP(AP_SSID, AP_PASS);
        espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);
        Serial.println("[SyncTeach] ESP-NOW start failed, fallback to AP");
        return false;
    }

    espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);
    espnow_ctrl.enable_sync(false);
    current_system_mode = MODE_ESPNOW_RUNNING;
    return true;
}

static void enter_sync_teach_mode() {
    if (!sync_teach_ensure_espnow_running()) {
        sync_teach_beep(2);
        return;
    }

    action_clear_requests(ACTION_REQ_ALL);
    action_recording = false;
    action_playback_active = false;
    action_edit_mode = false;
    action_set_servo_stream(false);

    sync_teach_mode = true;
    sync_teach_recording = false;
    sync_teach_playback_active = false;
    sync_teach_overflow = false;
    sync_teach_auto_stop_pending = false;
    sync_teach_playback_index = 0;
    espnow_ctrl.enable_sync(false);
    sync_teach_set_realtime_servo_mode(false);
    arm.set_torque(true);

    arm.board.oled.set_custom_text(0, "Sync Teach");
    arm.board.oled.set_custom_text(1, "ESP-NOW Ready");
    arm.board.oled.set_custom_text(2, "Record: OFF");
    arm.board.oled.set_custom_text(3, "Slave Hold");
    current_oled_mode = SHOW_CUSTOM;
    arm.board.oled.show_custom();

    sync_teach_beep(1);
    Serial.println("[SyncTeach] Entered mode");
}

static void exit_sync_teach_mode() {
    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_recording = false;
    sync_teach_playback_active = false;
    sync_teach_auto_stop_pending = false;
    portEXIT_CRITICAL(&sync_teach_mux);

    sync_teach_close_record_file();
    sync_teach_playback_index = 0;
    espnow_ctrl.enable_sync(false);
    sync_teach_set_realtime_servo_mode(false);
    arm.set_torque(true);

    sync_teach_mode = false;
    current_oled_mode = SHOW_NET_INFO;
    arm.move(200.0f, 0.0f, 200.0f, 0.0f, 0.0f, 0.0f, 1000);
    sync_teach_beep(1);
    Serial.println("[SyncTeach] Exited mode");
}

static void start_sync_teach_recording() {
    if (!sync_teach_ensure_espnow_running()) {
        sync_teach_beep(2);
        return;
    }

    sync_teach_frames_clear();

    if (!sync_teach_open_record_file()) {
        sync_teach_beep(2);
        return;
    }

    sync_teach_set_realtime_servo_mode(true);
    arm.set_torque(true);

    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_recording = true;
    sync_teach_playback_active = false;
    sync_teach_overflow = false;
    sync_teach_auto_stop_pending = false;
    sync_teach_have_first_seq = false;
    sync_teach_first_seq = 0;
    sync_teach_first_arrival_ms = 0;
    sync_teach_last_timestamp = 0;
    portEXIT_CRITICAL(&sync_teach_mux);

    espnow_ctrl.enable_sync(true);

    arm.board.oled.set_custom_text(0, "Sync Recording");
    arm.board.oled.set_custom_text(1, "Master stream");
    arm.board.oled.set_custom_text(2, "Slave following");
    arm.board.oled.set_custom_text(3, "");
    arm.board.oled.show_custom();

    sync_teach_beep(2);
    Serial.println("[SyncTeach] Recording started");
}

static void stop_sync_teach_recording_and_save() {
    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_recording = false;
    sync_teach_auto_stop_pending = false;
    portEXIT_CRITICAL(&sync_teach_mux);

    sync_teach_flush_record_file_all();
    sync_teach_close_record_file();
    espnow_ctrl.enable_sync(false);
    sync_teach_set_realtime_servo_mode(false);
    arm.set_torque(true);
    sync_teach_beep(3);

    uint16_t count = sync_teach_frame_count;
    uint32_t duration = sync_teach_last_timestamp;

    arm.board.oled.set_custom_text(0, sync_teach_overflow ? "Saved Partial" : "Saved!");
    arm.board.oled.set_custom_text(1, String("Frames: ") + count);
    arm.board.oled.set_custom_text(2, String("Time: ") + duration + "ms");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();

    Serial.printf("[SyncTeach] Recording stopped, saved %u frames, duration %lu ms, overflow=%d\n",
                  count, duration, sync_teach_overflow ? 1 : 0);
}

static void sync_teach_service_during_playback() {
    serial_port.rec_handler();
    if (current_system_mode == MODE_WIFI_AP_RUNNING) {
        espnow_ctrl.server_loop();
    }
    pump_at32_feedback();

    uint32_t stop_flags = sync_teach_take_requests(SYNC_TEACH_REQ_STOP_PLAY | SYNC_TEACH_REQ_EXIT);
    if (stop_flags & SYNC_TEACH_REQ_STOP_PLAY) {
        sync_teach_playback_active = false;
    }
    if (stop_flags & SYNC_TEACH_REQ_EXIT) {
        sync_teach_queue_request(SYNC_TEACH_REQ_EXIT);
        sync_teach_playback_active = false;
    }
}

static void sync_teach_wait_until(uint32_t due_time) {
    while (sync_teach_playback_active && (int32_t)(due_time - millis()) > 0) {
        sync_teach_service_during_playback();
        int32_t remaining = (int32_t)(due_time - millis());
        if (remaining > 2) {
            vTaskDelay(pdMS_TO_TICKS(SYNC_TEACH_PLAYBACK_SERVICE_MS));
        } else if (remaining > 0) {
            delayMicroseconds(200);
        }
    }
}

static void start_sync_teach_playback() {
    sync_teach_close_record_file();

    File file = LittleFS.open(SYNC_TEACH_RECORD_FILE, "rb");
    if (!file) {
        Serial.println("[SyncTeach] No sync teach action to play");
        sync_teach_beep(2);
        return;
    }

    uint16_t count = 0;
    if (file.read((uint8_t*)&count, sizeof(uint16_t)) != sizeof(uint16_t) || count < 2) {
        file.close();
        Serial.println("[SyncTeach] Too few frames to play");
        sync_teach_beep(2);
        return;
    }

    sync_teach_playback_active = true;
    sync_teach_playback_index = 0;
    espnow_ctrl.enable_sync(false);
    arm.set_torque(true);
    sync_teach_set_realtime_servo_mode(true);

    sync_teach_frame_count = count;
    arm.board.oled.set_custom_text(0, "Sync Playing");
    arm.board.oled.set_custom_text(1, String("Frames: ") + count);
    arm.board.oled.set_custom_text(2, "Torque: ON");
    arm.board.oled.set_custom_text(3, "");
    arm.board.oled.show_custom();

    sync_teach_beep(2);
    Serial.printf("[SyncTeach] Playback started (%u frames)\n", count);

    SyncTeachFrame first_frame;
    if (file.read((uint8_t*)first_frame.master_pos, 6 * sizeof(int16_t)) != 6 * sizeof(int16_t) ||
        file.read((uint8_t*)&first_frame.timestamp, sizeof(uint32_t)) != sizeof(uint32_t)) {
        file.close();
        sync_teach_playback_active = false;
        sync_teach_set_realtime_servo_mode(false);
        arm.set_torque(true);
        sync_teach_beep(2);
        Serial.println("[SyncTeach] Failed to read first frame");
        return;
    }

    espnow_ctrl.applyMasterTeachPositions(first_frame.master_pos, 0);
    uint32_t playback_start = millis();
    uint32_t first_timestamp = first_frame.timestamp;
    bool playback_completed = true;

    for (uint16_t frame_idx = 1; sync_teach_playback_active && frame_idx < count; frame_idx++) {
        SyncTeachFrame frame;
        if (file.read((uint8_t*)frame.master_pos, 6 * sizeof(int16_t)) != 6 * sizeof(int16_t) ||
            file.read((uint8_t*)&frame.timestamp, sizeof(uint32_t)) != sizeof(uint32_t)) {
            playback_completed = false;
            break;
        }

        uint32_t frame_time = frame.timestamp - first_timestamp;
        uint32_t due_time = playback_start + frame_time;
        sync_teach_wait_until(due_time);

        if (!sync_teach_playback_active) {
            playback_completed = false;
            break;
        }

        espnow_ctrl.applyMasterTeachPositions(frame.master_pos, 0);
        sync_teach_playback_index = frame_idx;
    }

    file.close();

    if (!sync_teach_playback_active) {
        playback_completed = false;
    }

    sync_teach_playback_active = false;
    sync_teach_playback_index = 0;
    sync_teach_set_realtime_servo_mode(false);
    arm.set_torque(true);

    if (playback_completed) {
        arm.board.oled.set_custom_text(0, "Sync Play Done");
        arm.board.oled.set_custom_text(1, String("Frames: ") + count);
        arm.board.oled.set_custom_text(2, "Torque: ON");
        arm.board.oled.set_custom_text(3, "Ready");
        arm.board.oled.show_custom();
        Serial.println("[SyncTeach] Playback completed");
    } else {
        Serial.println("[SyncTeach] Playback stopped");
    }
}

static void stop_sync_teach_playback() {
    sync_teach_playback_active = false;
    sync_teach_playback_index = 0;
    sync_teach_set_realtime_servo_mode(false);
    arm.set_torque(true);
    sync_teach_beep(1);

    arm.board.oled.set_custom_text(0, "Sync Teach");
    arm.board.oled.set_custom_text(1, "Playback stopped");
    arm.board.oled.set_custom_text(2, "Torque: ON");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();
}

static void clear_sync_teach_data() {
    portENTER_CRITICAL(&sync_teach_mux);
    sync_teach_recording = false;
    sync_teach_playback_active = false;
    sync_teach_auto_stop_pending = false;
    sync_teach_overflow = false;
    portEXIT_CRITICAL(&sync_teach_mux);

    sync_teach_close_record_file();
    sync_teach_playback_index = 0;
    sync_teach_frames_clear();
    LittleFS.remove(SYNC_TEACH_RECORD_FILE);
    sync_teach_beep(2);

    arm.board.oled.set_custom_text(0, "Sync Cleared");
    arm.board.oled.set_custom_text(1, "No action");
    arm.board.oled.set_custom_text(2, "");
    arm.board.oled.set_custom_text(3, "Ready");
    arm.board.oled.show_custom();

    Serial.println("[SyncTeach] Data cleared");
}

static void handle_sync_teach_requests() {
    sync_teach_flush_record_file(SYNC_TEACH_FLUSH_BATCH);

    if (sync_teach_auto_stop_pending) {
        stop_sync_teach_recording_and_save();
    }

    uint32_t requests = sync_teach_take_requests(
        SYNC_TEACH_REQ_ENTER |
        SYNC_TEACH_REQ_EXIT |
        SYNC_TEACH_REQ_START |
        SYNC_TEACH_REQ_STOP |
        SYNC_TEACH_REQ_PLAY |
        SYNC_TEACH_REQ_STOP_PLAY |
        SYNC_TEACH_REQ_CLEAR
    );

    if (!requests) {
        return;
    }

    if (requests & SYNC_TEACH_REQ_STOP_PLAY) {
        if (sync_teach_playback_active) {
            stop_sync_teach_playback();
        }
    }

    if (requests & SYNC_TEACH_REQ_STOP) {
        if (sync_teach_recording) {
            stop_sync_teach_recording_and_save();
        }
    }

    if (requests & SYNC_TEACH_REQ_CLEAR) {
        if (!sync_teach_recording && !sync_teach_playback_active) {
            clear_sync_teach_data();
        }
    }

    if (requests & SYNC_TEACH_REQ_ENTER) {
        enter_sync_teach_mode();
    }

    if (requests & SYNC_TEACH_REQ_EXIT) {
        exit_sync_teach_mode();
        return;
    }

    if ((requests & SYNC_TEACH_REQ_START) && sync_teach_mode && !sync_teach_recording && !sync_teach_playback_active) {
        start_sync_teach_recording();
    }

    if ((requests & SYNC_TEACH_REQ_PLAY) && sync_teach_mode && !sync_teach_recording && !sync_teach_playback_active) {
        start_sync_teach_playback();
    }
}

static bool sync_teach_blocks_action_commands() {
    return sync_teach_mode ||
           sync_teach_recording ||
           sync_teach_playback_active ||
           sync_teach_has_request(SYNC_TEACH_REQ_ENTER | SYNC_TEACH_REQ_START | SYNC_TEACH_REQ_PLAY);
}

static void user_button_sub_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    uint32_t now = millis();

    if(now - user_button_last_press > 10000) {
        user_button_press_count = 0;
    }

    user_button_last_press = now;
    user_button_press_count++;

    if(user_button_press_count == 1) {
        if(current_bt_mode == BT_MODE_BLE) {
            arm.board.oled.set_custom_text(0, "Switch to PS3");
            arm.board.oled.set_custom_text(1, "Press again to");
            arm.board.oled.set_custom_text(2, "confirm switch");
            arm.board.oled.set_custom_text(3, "");
        } else {
            arm.board.oled.set_custom_text(0, "Switch to BLE");
            arm.board.oled.set_custom_text(1, "Press again to");
            arm.board.oled.set_custom_text(2, "confirm switch");
            arm.board.oled.set_custom_text(3, "");
        }
        current_oled_mode = SHOW_CUSTOM;
        arm.board.oled.show_custom();

        ps3_prompt_active = true;
        ps3_prompt_start_time = now;

    } else if(user_button_press_count == 2) {
        user_button_press_count = 0;
        ps3_prompt_active = false;

        BluetoothMode new_mode = (current_bt_mode == BT_MODE_BLE) ? BT_MODE_PS3 : BT_MODE_BLE;
        current_bt_mode = new_mode;
        save_config();

        uint8_t response[2] = {new_mode, 1};
        uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_BT_MODE, response, 2);
        serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);

        Serial.printf("Switching to %s mode...\n", new_mode == BT_MODE_PS3 ? "PS3" : "BLE");
        Serial.flush();

        arm.board.oled.set_icon(4);
        current_oled_mode = SHOW_ICON;

        arm.board.buzzer.off();
        noTone(23);
        ledcDetachPin(23);

        uint32_t start = millis();
        while (millis() - start < 1000) {
            arm.board.oled.show_icon();
            delay(30);
        }

        ESP.restart();
    }
}

void at32_packet_callback(PacketTypeDef* rx_packet)
{
    if(is_downloading) return;

    if (rx_packet->elements.id == 0xFF &&
        rx_packet->elements.cmd == CMD_SERVO_STREAM &&
        rx_packet->elements.length >= 14) {
        for (int i = 0; i < 6; ++i) {
            int idx = i * 2;
            g_last_servo_positions[i] = (int16_t)((rx_packet->elements.args[idx + 1] << 8) | rx_packet->elements.args[idx]);
        }
        g_last_servo_feedback_ms = millis();
        g_last_servo_stream_ms = g_last_servo_feedback_ms;
        if (action_recording && action_servo_stream_active) {
            memcpy(last_recorded_pos, g_last_servo_positions, sizeof(last_recorded_pos));
            action_record_feedback_frame(g_last_servo_positions);
            action_print_json("record", g_last_servo_positions, recorded_frame_count, recorded_frame_count);
        }
        return;
    }

    if (action_recording && action_servo_stream_active) {
        return;
    }

    if (rx_packet->elements.id == 0xFF &&
        rx_packet->elements.cmd == 96 &&
        rx_packet->elements.length >= 14) {
        for (int i = 0; i < 6; ++i) {
            int idx = i * 2;
            g_last_servo_positions[i] = (int16_t)((rx_packet->elements.args[idx + 1] << 8) | rx_packet->elements.args[idx]);
        }
        g_last_servo_feedback_ms = millis();
        return;
    }

    if (rx_packet->elements.id == 0x5A) {
        if (rx_packet->elements.cmd == CMD_GET_CUR_COORDS ||
            rx_packet->elements.cmd == CMD_IKINE_RESULT_GET ||
            rx_packet->elements.cmd == CMD_FKINE_RESULT_GET ||
            rx_packet->elements.cmd == CMD_GET_POS_OFFSET ||
            rx_packet->elements.cmd == CMD_GET_PID_PARAM ||
            rx_packet->elements.cmd == CMD_GET_REAL_JOINT_ANGLES ||
            rx_packet->elements.cmd == CMD_GET_REAL_TCP_POSE ||
            rx_packet->elements.cmd == CMD_SERVO_READ_OVERLOAD ||
            rx_packet->elements.cmd == CMD_SERVO_READ_BAUD ||
            rx_packet->elements.cmd == CMD_SERVO_WRITE_BAUD ||
            rx_packet->elements.cmd == CMD_SERVO_READ_MAX_TORQUE ||
            rx_packet->elements.cmd == CMD_SERVO_READ_ANGLE_LIMIT ||
            rx_packet->elements.cmd == CMD_GET_COORD_LIMITS) {

            if (rx_packet->elements.cmd == CMD_GET_CUR_COORDS && rx_packet->elements.length >= 26) {
                int16_t raw_x = (int16_t)((rx_packet->elements.args[1] << 8) | rx_packet->elements.args[0]);
                int16_t raw_y = (int16_t)((rx_packet->elements.args[3] << 8) | rx_packet->elements.args[2]);
                int16_t raw_z = (int16_t)((rx_packet->elements.args[5] << 8) | rx_packet->elements.args[4]);
                int16_t raw_p = (int16_t)((rx_packet->elements.args[7] << 8) | rx_packet->elements.args[6]);
                int16_t raw_r = (int16_t)((rx_packet->elements.args[9] << 8) | rx_packet->elements.args[8]);
                int16_t raw_c = (int16_t)((rx_packet->elements.args[11] << 8) | rx_packet->elements.args[10]);

                arm.current_pose.x = (float)raw_x;
                arm.current_pose.y = (float)raw_y;
                arm.current_pose.z = (float)raw_z;
                arm.current_pose.pitch = (float)raw_p / 10.0f;
                arm.current_pose.roll = (float)raw_r;
                arm.current_pose.claw = (float)raw_c;

                for (int i = 0; i < 6; ++i) {
                    int idx = 12 + i * 2;
                    g_last_servo_positions[i] = (int16_t)((rx_packet->elements.args[idx + 1] << 8) | rx_packet->elements.args[idx]);
                }
                g_last_servo_feedback_ms = millis();
            }

            uint8_t data_len = rx_packet->elements.length - 2;
            uint8_t send_buf[32];
            if (data_len > sizeof(send_buf)) {
                data_len = sizeof(send_buf);
            }
            memcpy(send_buf, rx_packet->elements.args, data_len);
            uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, rx_packet->elements.cmd, send_buf, data_len);
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
            ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
        }
    }
}
void pump_at32_feedback(void)
{
    while(servo.uart->available()) {
        uint8_t c = servo.uart->read();
        at32_protocol.parsing(&c, 1);
    }
}

bool sync_arm_feedback(uint32_t timeout_ms)
{
    arm.update_status();
    uint32_t start = millis();
    uint32_t prev_feedback_ms = g_last_servo_feedback_ms;

    while(millis() - start < timeout_ms) {
        serial_port.rec_handler();
        pump_at32_feedback();
        if(g_last_servo_feedback_ms != 0 && g_last_servo_feedback_ms != prev_feedback_ms) {
            return true;
        }
        delay(2);
    }

    return g_last_servo_feedback_ms != 0;
}

bool get_last_servo_positions(int16_t out_pos[6])
{
    if(g_last_servo_feedback_ms == 0) {
        return false;
    }

    memcpy(out_pos, g_last_servo_positions, sizeof(g_last_servo_positions));
    return true;
}

void func_ctrl_callback(PacketTypeDef* self)
{
    uint8_t len;
    uint16_t bat_level;

    if (!lerobotBridgeMode) {
        Serial.printf("[CB] id=0x%02X cmd=%d len=%d\n",
                      self->elements.id, self->elements.cmd, self->elements.length);
    }

    if(self->elements.id == 0xFF && self->elements.cmd == CMD_ACTION_GROUP_DOWNLOAD) {
        is_downloading = true;
        arm.board.action_group_download(self->elements.args[0], self->elements.args, self->elements.length - 2);
        is_downloading = false;
        return;
    }

    if (action_should_block_external_servo_command(self)) {
        action_blocked_cmd_count++;
        return;
    }

    if(self->elements.id == 0xFF) {
        switch (self->elements.cmd)
        {
            case CMD_FIRMWARE_VERSION_CHECK: {

                uint8_t at32_ver[3] = {0, 0, 0};
                AT32_OTA::query_version(at32_protocol, *servo.uart, at32_ver);

                uint8_t ver[6] = {
                    FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
                    at32_ver[0], at32_ver[1], at32_ver[2]
                };
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_FIRMWARE_VERSION_CHECK, ver, 6);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                Serial.printf("[FW] ESP32=%d.%d.%d AT32=%d.%d.%d\n",
                              ver[0], ver[1], ver[2], ver[3], ver[4], ver[5]);
            } break;

            case CMD_CHECK_BAT_LEVEL_CHECK:
                bat_level = (uint16_t)arm.board.bat.get_voltage();
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_CHECK_BAT_LEVEL_CHECK, (uint8_t*)&bat_level, sizeof(bat_level));
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                break;

            case CMD_ACTION_GROUP_RUN:
                arm.board.action_group_run(self->elements.args[0]);
                break;

            case CMD_SET_KINEMATICS_PARAM: {
                if (self->elements.length - 2 >= sizeof(arm_kin)) {
                    memcpy(arm_kin, self->elements.args, sizeof(arm_kin));
                    save_config();
                    servo.tx_frame_write(0xFF, CMD_SET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));
                    arm.board.buzzer.set(100, 100, 1, 2000);
                    Serial.printf("[Kin] Updated base_high=%.1f\n", arm_kin[8]);
                }
            } break;

            case CMD_GET_KINEMATICS_PARAM: {
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_GET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
            } break;

            case CMD_SET_POS_OFFSET:
                case CMD_GET_POS_OFFSET:
                case CMD_SET_PID_PARAM:
                case CMD_GET_PID_PARAM:

                    servo.tx_frame_write(0xFF, self->elements.cmd, self->elements.args, self->elements.length - 2);
                    break;
            case CMD_GET_CUR_COORDS:
                servo.tx_frame_write(0xFF, CMD_GET_CUR_COORDS, self->elements.args, self->elements.length - 2);
                break;

            case CMD_ACTION_GROUP_STOP: {
                arm.board.action_group_stop();

                uint8_t ack = 1;
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_ACTION_GROUP_STOP, &ack, 1);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                ble_port.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
                espnow_ctrl.sendData((uint8_t*)&serial_port.protocol.tx_packet, len);
            } break;

            case CMD_ACTION_GROUP_ERASE:
                arm.board.action_group_erase(self->elements.args[0]);
                break;

            case CMD_BUZZER_SET: {
                uint32_t on_t  = (uint32_t)self->elements.args[0] | ((uint32_t)self->elements.args[1] << 8) | ((uint32_t)self->elements.args[2] << 16) | ((uint32_t)self->elements.args[3] << 24);
                uint32_t off_t = (uint32_t)self->elements.args[4] | ((uint32_t)self->elements.args[5] << 8) | ((uint32_t)self->elements.args[6] << 16) | ((uint32_t)self->elements.args[7] << 24);
                uint16_t cnt   = (uint16_t)self->elements.args[8] | ((uint16_t)self->elements.args[9] << 8);
                uint16_t freq  = (uint16_t)self->elements.args[10] | ((uint16_t)self->elements.args[11] << 8);
                arm.board.buzzer.set(on_t, off_t, cnt, freq);
            } break;

            case CMD_OLED_SET:
                if (self->elements.length >= 4) {
                    uint8_t line = self->elements.args[0];
                    uint8_t txt_len = self->elements.args[1];
                    String text = "";
                    for(int i = 0; i < txt_len; i++) {
                        text += (char)self->elements.args[2 + i];
                    }
                    arm.board.oled.set_custom_text(line, text);
                    current_oled_mode = SHOW_CUSTOM;
                }
                break;

            case CMD_COORDINATE_SET:
                servo.tx_frame_write(0xFF, CMD_COORDINATE_SET, self->elements.args, self->elements.length - 2);

                if (self->elements.length - 2 >= 14) {
                    int16_t p_raw = (int16_t)((self->elements.args[1] << 8) | self->elements.args[0]);
                    int16_t x_raw = (int16_t)((self->elements.args[3] << 8) | self->elements.args[2]);
                    int16_t y_raw = (int16_t)((self->elements.args[5] << 8) | self->elements.args[4]);
                    int16_t z_raw = (int16_t)((self->elements.args[7] << 8) | self->elements.args[6]);
                    int16_t r_raw = (int16_t)((self->elements.args[9] << 8) | self->elements.args[8]);
                    int16_t c_raw = (int16_t)((self->elements.args[11] << 8) | self->elements.args[10]);

                    arm.current_pose.x = (float)x_raw;
                    arm.current_pose.y = (float)y_raw;
                    arm.current_pose.z = (float)z_raw;
                    arm.current_pose.pitch = (float)p_raw / 10.0f;
                    arm.current_pose.roll = (float)r_raw;
                    arm.current_pose.claw = (float)c_raw;
                }
                break;

            case CMD_FKINE_RESULT_GET:
            case CMD_IKINE_RESULT_GET:
                servo.tx_frame_write(0xFF, self->elements.cmd, self->elements.args, self->elements.length - 2);
                break;

            case CMD_SET_ESPNOW_CHANNEL:
                global_channel = self->elements.args[0];
                save_config();

                if (current_system_mode == MODE_ESPNOW_RUNNING) {
                    espnow_ctrl.stop();
                    delay(50);
                    if (espnow_ctrl.begin(global_channel)) {
                        espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);
                    } else {
                        current_system_mode = MODE_WIFI_AP_RUNNING;
                        espnow_ctrl.startAP(AP_SSID, AP_PASS);
                        espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);
                        Serial.println("[ESP-NOW] restart failed after channel change, fallback to AP");
                    }
                }
                arm.board.buzzer.set(100, 100, 1, 2000);
                Serial.printf("[Config] ESP-NOW channel set to %d\n", global_channel);
                break;

            case CMD_ESPNOW_SYNC_CTRL: {
                if (current_system_mode != MODE_ESPNOW_RUNNING) break;

                bool enable = (self->elements.args[0] == 1);

                if(enable) {
                    espnow_ctrl.enable_sync(true);
                    arm.board.buzzer.set(100, 100, 2, 2000);

                    uint8_t t_data[2];
                    t_data[0] = 40;
                    t_data[1] = 1;
                    for(int i=0; i<3; i++) {
                        servo.tx_frame_write(0xFE, 3, t_data, 2);
                        delay(5);
                    }

                    uint8_t acc_data[2];
                    acc_data[0] = 41;
                    acc_data[1] = global_acc;
                    servo.tx_frame_write(0xFE, 3, acc_data, 2);

                } else {
                    espnow_ctrl.sync_enabled = false;
                    espnow_ctrl.enable_sync(false);
                    arm.board.buzzer.set(500, 200, 1, 1000);

                    uint8_t t_data[2] = {40, 0};
                    servo.tx_frame_write(0xFE, 3, t_data, 2);
                }
            } break;

            case CMD_SET_PEER_MAC:
                if(self->elements.length >= 6) {
                    espnow_ctrl.set_peering_mac(self->elements.args);
                    arm.board.buzzer.set(100, 100, 1, 3000);
                }
                break;

            case CMD_SET_GLOBAL_ACC:
                global_acc = self->elements.args[0];
                save_config();

                {

                    uint8_t acc_data[2];
                    acc_data[0] = 41;
                    acc_data[1] = global_acc;
                    servo.tx_frame_write(BROADCAST_ID, 3, acc_data, 2);

                    delay(5);
                    servo.tx_frame_write(0xFF, CMD_SET_MOVE_ACC, &global_acc, 1);
                }
                arm.board.buzzer.set(500, 200, 1, 1000);
                Serial.printf("[Config] Global ACC set to %d\n", global_acc);
                break;
            case CMD_SET_SINGLE_MOTOR:
                if(self->elements.length >= 4) {
                    uint8_t motor_idx = self->elements.args[0];
                    int8_t motor_spd = (int8_t)self->elements.args[1];
                    arm.board.motor.set_single_speed(motor_idx, motor_spd);
                }
                break;
            case CMD_STOP_ALL_MOTOR:
                arm.board.motor.stop();
                break;
            case CMD_SET_MOTOR_SPEED:
                if(self->elements.length >= 6) {
                    int8_t s1 = (int8_t)self->elements.args[0];
                    int8_t s2 = (int8_t)self->elements.args[1];
                    int8_t s3 = (int8_t)self->elements.args[2];
                    int8_t s4 = (int8_t)self->elements.args[3];
                    arm.board.motor.set_speed(s1, s2, s3, s4);
                }
                break;
            case CMD_MECANUM_CONTROL: {
                if (self->elements.length >= 5) {
                    int8_t vx = (int8_t)self->elements.args[0];
                    int8_t vy = (int8_t)self->elements.args[1];
                    int8_t vz = (int8_t)self->elements.args[2];
                    if (chassis_type == CHASSIS_TANK) vy = 0;
                    arm.board.mecanum.set_velocity(vx, vy, vz);
                }
                break;
            }

            case CMD_TANK_CONTROL: {
                if (self->elements.length >= 4) {
                    int8_t speed = (int8_t)self->elements.args[0];
                    int8_t turn  = (int8_t)self->elements.args[1];
                    arm.board.tank.run(speed, turn);
                }
                break;
            }
            case CMD_CONVEYOR_SET:
                if(self->elements.length >= 3) {
                    int8_t c_speed = (int8_t)self->elements.args[0];
                    arm.board.conveyor.set_speed(c_speed);
                }
                break;
            case CMD_STEPPER_RESET:
                arm.board.stepper.reset();
                break;

           case CMD_STEPPER_DIV:
                if(self->elements.length >= 3) {
                    uint8_t code = self->elements.args[0];
                    arm.board.stepper.set_subdivision(code);
                }
                break;
            case CMD_STEPPER_RUN:
                if(self->elements.length >= 6) {
                    int32_t s_steps;
                    memcpy(&s_steps, self->elements.args, 4);
                    arm.board.stepper.move(s_steps);
                }
                break;
            case CMD_COLOR_TRACK:
                if(self->elements.length >= 4) {
                    uint8_t sw = self->elements.args[0];
                    uint8_t color_id = self->elements.args[1];

                    if (sw == 0) {
                        colorTrackerRot.stop();
                        colorGrabber.stop();
                        colorTrackerRot.stop();
                        arm.board.buzzer.off();
                    } else {
                        const char* cName = "";
                        if (color_id == 1) cName = "red";
                        else if (color_id == 2) cName = "green";
                        else if (color_id == 3) cName = "blue";

                        if (strlen(cName) > 0) {
                            faceTracker.stop();
                            selfLearningTracker.stop();
                            colorGrabber.stop();
                            aprilTagTracker.stop();
                            aprilTagGrabber.stop();
                            colorTrackerRot.stop();
                            garbageGrabber.stop();
                            gestureTracker.stop();
                            calibrationGrabber.stop();
                            aiLLMControl.stop();

                            delay(100);
                            colorTrackerRot.start(cName);

                            arm.board.buzzer.set(100, 100, 1, 2000);
                        }
                    }
                }
            break;
            case CMD_FACE_TRACK:
                if(self->elements.length >= 3) {
                    uint8_t sw = self->elements.args[0];
                    if (sw == 1) {
                        selfLearningTracker.stop();
                        aprilTagTracker.stop();
                        aprilTagGrabber.stop();
                        colorTrackerRot.stop();
                        aiLLMControl.stop();
                        garbageGrabber.stop();
                        faceTracker.stop();
                        arm.board.action_group_stop();
                        espnow_ctrl.sync_enabled = false;
                        espnow_ctrl.enable_sync(false);

                        delay(100);
                        faceTracker.start();
                        arm.board.buzzer.set(100, 100, 1, 2000);
                        Serial.println("[SYS] FACE_TRACK ON");
                    } else {
                        faceTracker.stop();
                        arm.board.buzzer.off();
                        Serial.println("[SYS] FACE_TRACK OFF");
                        }
                    }
                    break;
            case CMD_SELF_LEARN_TRACK:
                if(self->elements.length >= 3) {
                    uint8_t sw = self->elements.args[0];

                    if (sw == 1) {
                        faceTracker.stop();
                        aiLLMControl.stop();
                        aprilTagTracker.stop();
                        aprilTagGrabber.stop();
                        colorTrackerRot.stop();
                        selfLearningTracker.stop();
                        arm.board.action_group_stop();

                        delay(100);
                        selfLearningTracker.start();
                        arm.board.buzzer.set(100, 100, 1, 2000);
                    }
                    else if (sw == 2) {
                        int cx = 160;
                        int cy = 120;
                        String id = "1";

                        if (self->elements.length >= 7) {
                            cx = (int16_t)((self->elements.args[1] << 8) | self->elements.args[2]);
                            cy = (int16_t)((self->elements.args[3] << 8) | self->elements.args[4]);

                            uint8_t n_len = self->elements.args[5];
                            if (n_len > 0 && self->elements.length >= 7 + n_len) {
                                char nameBuf[32] = {0};
                                int copy_len = (n_len < 31) ? n_len : 31;
                                memcpy(nameBuf, &self->elements.args[6], copy_len);
                                id = String(nameBuf);
                            }
                        }
                        selfLearningTracker.confirm(id, cx, cy);
                        arm.board.buzzer.set(100, 50, 2, 3000);
                    }
                    else {
                        selfLearningTracker.stop();
                        arm.board.buzzer.off();
                    }
                }
                break;
            case CMD_APRILTAG_TRACK:
                if(self->elements.length >= 3) {
                    uint8_t sw = self->elements.args[0];
                    Serial.printf("[SYS] CMD_APRILTAG_TRACK sw=%d\n", sw);

                    if (sw == 1) {
                        faceTracker.stop();
                        selfLearningTracker.stop();
                        aiLLMControl.stop();
                        aprilTagGrabber.stop();
                        colorTrackerRot.stop();
                        aprilTagTracker.stop();
                        arm.board.action_group_stop();

                        delay(100);
                        aprilTagTracker.start();

                        arm.board.buzzer.set(100, 100, 1, 2000);
                    } else {
                        aprilTagTracker.stop();
                        arm.board.buzzer.off();
                    }
                }
                break;

            case CMD_APRILTAG_GRAB:
            if(self->elements.length >= 3) {
                uint8_t sw = self->elements.args[0];
                if (sw == 1) {
                    faceTracker.stop();
                    aprilTagTracker.stop();
                    aprilTagGrabber.stop();
                    selfLearningTracker.stop();
                    colorTrackerRot.stop();
                    aiLLMControl.stop();
                    arm.board.action_group_stop();

                    delay(100);
                    aprilTagGrabber.start();
                    arm.board.buzzer.set(100, 100, 2, 2000);
                } else {
                    aprilTagGrabber.stop();
                    arm.board.buzzer.off();
                }
            }
            break;

            case CMD_APRILTAG_SET_OFFSET:
            if(self->elements.length >= 14) {
                float x_off, y_off, z_grab;
                memcpy(&x_off, &self->elements.args[0], 4);
                memcpy(&y_off, &self->elements.args[4], 4);
                memcpy(&z_grab, &self->elements.args[8], 4);
                colorGrabber.setOffsets(x_off, y_off, z_grab);

                aprilTagGrabber.setOffsets(x_off, y_off, z_grab);
                garbageGrabber.setOffsets(x_off, y_off, z_grab);
                calibrationGrabber.setOffsets(x_off, y_off, z_grab);
                colorGrabber.setOffsets(x_off, y_off, z_grab);

                prefs.begin("robot_cfg", false);
                prefs.putFloat("ag_x", x_off);
                prefs.putFloat("ag_y", y_off);
                prefs.putFloat("ag_z", z_grab);
                prefs.end();

                arm.board.buzzer.set(100, 50, 2, 3000);
            }
            break;
            case CMD_COLOR_GRAB:
            if(self->elements.length >= 4) {
                uint8_t sw = self->elements.args[0];
                uint8_t color_id = self->elements.args[1];

                if (sw == 0) {
                    colorGrabber.stop();
                    colorTrackerRot.stop();
                    arm.board.buzzer.off();
                } else {
                    const char* cName = "";
                    if (color_id == 1) cName = "red";
                    else if (color_id == 2) cName = "green";
                    else if (color_id == 3) cName = "blue";

                    Serial.printf("[SYS] CMD_COLOR_GRAB sw=%d color=%d name='%s'\n", sw, color_id, cName);

                    if (strlen(cName) > 0) {
                        colorGrabber.stop();
                        colorTrackerRot.stop();
                        faceTracker.stop();
                        aprilTagTracker.stop();
                        aprilTagGrabber.stop();
                        selfLearningTracker.stop();
                        aiLLMControl.stop();
                        gestureTracker.stop();
                        calibrationGrabber.stop();
                        garbageGrabber.stop();
                        arm.board.action_group_stop();
                        espnow_ctrl.sync_enabled = false;
                        espnow_ctrl.enable_sync(false);

                        delay(100);

                        const char* colors[] = { cName };
                        colorGrabber.start(colors, 1);
                        arm.board.buzzer.set(100, 100, 2, 2000);
                    }
                }

            }
            break;

            case CMD_LLM_CONTROL:
            if(self->elements.length >= 3) {
                uint8_t sw = self->elements.args[0];
                Serial.printf("[SYS] CMD_LLM_CONTROL sw=%d len=%d\n", sw, self->elements.length);
                if (sw == 1) {
                    faceTracker.stop();
                    selfLearningTracker.stop();
                    aprilTagTracker.stop();
                    aprilTagGrabber.stop();
                    colorTrackerRot.stop();
                    aiLLMControl.stop();
                    arm.board.action_group_stop();

                    delay(100);
                    aiLLMControl.start();
                    arm.board.buzzer.set(100, 100, 1, 2000);
                } else if (sw == 0) {
                    Serial.println("[SYS] CMD_LLM_CONTROL sw=0 ignored, MCP stays auto-registered");
                    arm.board.buzzer.off();
                } else {
                    Serial.printf("[SYS] CMD_LLM_CONTROL ignore invalid sw=%d\n", sw);
                }
            }
            break;

            case CMD_GESTURE_TRACK:
            Serial.printf("[SYS] CMD_GESTURE_TRACK len=%d\n", self->elements.length);
            if(self->elements.length >= 3) {
                uint8_t sw = self->elements.args[0];
                if (sw == 1) {
                    faceTracker.stop();
                    selfLearningTracker.stop();
                    aprilTagTracker.stop();
                    aprilTagGrabber.stop();
                    colorTrackerRot.stop();
                    colorGrabber.stop();
                    aiLLMControl.stop();
                    garbageGrabber.stop();
                    calibrationGrabber.stop();
                    gestureTracker.stop();
                    arm.board.action_group_stop();
                    espnow_ctrl.sync_enabled = false;
                    espnow_ctrl.enable_sync(false);

                    delay(100);

                    gestureTracker.start();
                    arm.board.buzzer.set(100, 100, 1, 2000);
                } else {
                    gestureTracker.stop();
                    arm.board.buzzer.off();
                }
            }
            break;

            case CMD_OLED_ICON:
            if (self->elements.length >= 3) {
                uint8_t icon_id = self->elements.args[0];
                arm.board.oled.set_icon(icon_id);
                current_oled_mode = SHOW_ICON;
            }
            break;
            case CMD_GARBAGE_GRAB:
            if(self->elements.length >= 3) {
                uint8_t sw = self->elements.args[0];

                if(self->elements.length >= 4) {
                    uint8_t thresh = self->elements.args[1];
                    if (thresh > 0 && thresh <= 100) {
                        garbageGrabber.setThreshold(thresh);
                    }
                }

                if (sw == 1) {
                    faceTracker.stop();
                    selfLearningTracker.stop();
                    aprilTagTracker.stop();
                    aprilTagGrabber.stop();
                    colorTrackerRot.stop();
                    aiLLMControl.stop();
                    garbageGrabber.stop();
                    arm.board.action_group_stop();

                    delay(100);
                    garbageGrabber.start();
                    arm.board.buzzer.set(100, 100, 2, 2000);
                } else {
                    garbageGrabber.stop();
                    arm.board.buzzer.off();
                }
            }
            break;

        case CMD_ARM_MOVE_INC: {
            servo.tx_frame_write(0xFF, CMD_ARM_MOVE_INC, self->elements.args, self->elements.length - 2);

            if(self->elements.length - 2 >= 14) {
                int16_t i_dx = (int16_t)((self->elements.args[1] << 8) | self->elements.args[0]);
                int16_t i_dy = (int16_t)((self->elements.args[3] << 8) | self->elements.args[2]);
                int16_t i_dz = (int16_t)((self->elements.args[5] << 8) | self->elements.args[4]);
                int16_t i_dp = (int16_t)((self->elements.args[7] << 8) | self->elements.args[6]);
                int16_t i_dr = (int16_t)((self->elements.args[9] << 8) | self->elements.args[8]);
                int16_t i_dc = (int16_t)((self->elements.args[11] << 8) | self->elements.args[10]);

                arm.current_pose.x += (float)i_dx;
                arm.current_pose.y += (float)i_dy;
                arm.current_pose.z += (float)i_dz;
                arm.current_pose.pitch += (float)i_dp / 10.0f;
                arm.current_pose.roll += (float)i_dr;
                arm.current_pose.claw += (float)i_dc;

                arm.current_pose.x     = ps3_clamp(arm.current_pose.x,     PS3_X_MIN,     PS3_X_MAX);
                arm.current_pose.y     = ps3_clamp(arm.current_pose.y,     PS3_Y_MIN,     PS3_Y_MAX);
                arm.current_pose.z     = ps3_clamp(arm.current_pose.z,     PS3_Z_MIN,     PS3_Z_MAX);
                arm.current_pose.pitch = ps3_clamp(arm.current_pose.pitch, PS3_PITCH_MIN, PS3_PITCH_MAX);
                arm.current_pose.roll  = ps3_clamp(arm.current_pose.roll,  PS3_ROLL_MIN,  PS3_ROLL_MAX);
                arm.current_pose.claw  = ps3_clamp(arm.current_pose.claw,  PS3_CLAW_MIN,  PS3_CLAW_MAX);
            }
        } break;

        case CMD_MOVE_INC: {

            if(self->elements.length - 2 >= 14) {
                int16_t i_dx = (int16_t)((self->elements.args[1] << 8) | self->elements.args[0]);
                int16_t i_dy = (int16_t)((self->elements.args[3] << 8) | self->elements.args[2]);
                int16_t i_dz = (int16_t)((self->elements.args[5] << 8) | self->elements.args[4]);
                int16_t i_dp = (int16_t)((self->elements.args[7] << 8) | self->elements.args[6]);
                int16_t i_dr = (int16_t)((self->elements.args[9] << 8) | self->elements.args[8]);
                int16_t i_dc = (int16_t)((self->elements.args[11] << 8) | self->elements.args[10]);
                uint16_t duration = (uint16_t)((self->elements.args[13] << 8) | self->elements.args[12]);

                float nx = ps3_clamp(arm.current_pose.x     + (float)i_dx,          PS3_X_MIN,     PS3_X_MAX);
                float ny = ps3_clamp(arm.current_pose.y     + (float)i_dy,          PS3_Y_MIN,     PS3_Y_MAX);
                float nz = ps3_clamp(arm.current_pose.z     + (float)i_dz,          PS3_Z_MIN,     PS3_Z_MAX);
                float np = ps3_clamp(arm.current_pose.pitch + (float)i_dp / 10.0f,  PS3_PITCH_MIN, PS3_PITCH_MAX);
                float nr = ps3_clamp(arm.current_pose.roll  + (float)i_dr,          PS3_ROLL_MIN,  PS3_ROLL_MAX);
                float nc = ps3_clamp(arm.current_pose.claw  + (float)i_dc,          PS3_CLAW_MIN,  PS3_CLAW_MAX);

                arm.move(nx, ny, nz, np, nr, nc, duration);
                Serial.printf("[SYS] MOVE_INC dx=%d dy=%d dz=%d dp=%d dr=%d dc=%d t=%d -> x=%.0f y=%.0f z=%.0f p=%.1f r=%.0f c=%.0f\n",
                              i_dx, i_dy, i_dz, i_dp, i_dr, i_dc, duration, nx, ny, nz, np, nr, nc);
            }
        } break;

        case CMD_SET_PS3_MAC: {

            if(self->elements.length - 2 >= 6) {
                char mac[18];
                snprintf(mac, sizeof(mac), "%02X:%02X:%02X:%02X:%02X:%02X",
                         self->elements.args[0], self->elements.args[1],
                         self->elements.args[2], self->elements.args[3],
                         self->elements.args[4], self->elements.args[5]);
                Serial.printf("[SYS] SET_PS3_MAC = %s, saving & restarting...\n", mac);

                strncpy(ps3_mac_buf, mac, sizeof(ps3_mac_buf) - 1);
                ps3_mac_buf[sizeof(ps3_mac_buf) - 1] = '\0';
                PS3_MAC = ps3_mac_buf;
                save_config();

                uint8_t resp[6];
                memcpy(resp, self->elements.args, 6);
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_PS3_MAC, resp, 6);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                Serial.flush();

                arm.board.buzzer.off();
                noTone(23);
                ledcDetachPin(23);
                delay(100);
                ESP.restart();
            }
        } break;
            case CMD_ARM_SERVO_SINGLE: {
                if(self->elements.length >= 5) {
                    uint8_t id = self->elements.args[0];
                    int16_t pos = (int16_t)((self->elements.args[2] << 8) | self->elements.args[1]);
                    uint16_t time_ms = (uint16_t)((self->elements.args[4] << 8) | self->elements.args[3]);

                    arm.move_servo(id, pos, time_ms);
                }
            } break;
            case CMD_SET_SERVO_ID:
                if (self->elements.length >= 4) {
                    uint8_t old_id = self->elements.args[0];
                    uint8_t new_id = self->elements.args[1];
                    arm.set_servo_id(old_id, new_id);
                }
                break;

            case CMD_SET_SERVO_MODE:
                if (self->elements.length >= 4) {
                    uint8_t id = self->elements.args[0];
                    uint8_t mode = self->elements.args[1];
                    arm.set_servo_mode(id, mode);
                }
                break;

            case CMD_ARM_RESET:
                if (self->elements.length >= 4) {
                    uint16_t duration = (uint16_t)((self->elements.args[1] << 8) | self->elements.args[0]);

                    arm.update_status();
                    delay(30);
                    arm.reset_all(duration);
                } else {
                    arm.update_status();
                    delay(30);
                    arm.reset_all(1000);
                }
                break;

             case CMD_SET_MOVE_ACC: {
                if (self->elements.length >= 3) {

                    servo.tx_frame_write(0xFF, CMD_SET_MOVE_ACC, self->elements.args, self->elements.length - 2);

                    uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_MOVE_ACC, self->elements.args, 1);
                    serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                }
            } break;

            case CMD_READ_ALL_SERVOS:
                arm.update_status();
                break;
            case CMD_SET_TORQUE: {
                if(self->elements.length >= 3) {
                    bool enable = (self->elements.args[0] != 0);
                    arm.set_torque(enable);
                }
            } break;
            case CMD_CALIBRATION:
                if(self->elements.length >= 3) {
                    uint8_t action = self->elements.args[0];
                    if (action == 1) {
                        faceTracker.stop(); selfLearningTracker.stop(); aprilTagTracker.stop(); aprilTagGrabber.stop(); colorTrackerRot.stop(); garbageGrabber.stop(); aiLLMControl.stop(); gestureTracker.stop(); arm.board.action_group_stop();
                        delay(100);
                        calibrationGrabber.start();
                        arm.board.buzzer.set(100, 100, 2, 2000);
                    } else if (action == 2) {
                        calibrationGrabber.grab();
                        arm.board.buzzer.set(100, 50, 1, 2000);
                    } else if (action == 3) {
                        calibrationGrabber.resetArm();
                        arm.board.buzzer.set(100, 50, 1, 2000);
                    } else if (action == 0) {
                        calibrationGrabber.stop();
                        arm.board.buzzer.off();
                    }
                }
                break;

            case CMD_SET_BT_MODE:
                if(self->elements.length >= 3) {
                    uint8_t new_mode = self->elements.args[0];
                    if(new_mode == BT_MODE_BLE || new_mode == BT_MODE_PS3) {
                        current_bt_mode = (BluetoothMode)new_mode;
                        save_config();

                        arm.board.buzzer.off();
                        noTone(23);
                        ledcDetachPin(23);

                        uint8_t response[2] = {new_mode, 1};
                        len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_BT_MODE, response, 2);
                        serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);

                        Serial.printf("Bluetooth mode set to: %s, restarting...\n",
                                    new_mode == BT_MODE_BLE ? "BLE" : "PS3");
                        Serial.flush();

                        arm.board.oled.set_icon(4);
                        current_oled_mode = SHOW_ICON;
                        uint32_t anim_start = millis();
                        while (millis() - anim_start < 1000) {
                            arm.board.oled.show_icon();
                            delay(30);
                        }
                        ESP.restart();
                    }
                }
                break;

            case CMD_LEROBOT_MODE: {
                if(self->elements.length >= 3) {
                    uint8_t enable = self->elements.args[0];
                    if (enable == 1) {

                        uint8_t resp = 1;
                        len = serial_port.protocol.tx_packet_complete(0xFF, CMD_LEROBOT_MODE, &resp, 1);
                        serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                        serial_port.uart->flush();
                        delay(50);

                        enter_lerobot_bridge_mode();
                    } else {

                        ESP.restart();
                    }
                }
            } break;

            case CMD_PC_SYNC_TEACH: {
                if ((self->elements.length - 2) >= 12) {
                    int16_t master_positions[6];
                    for (int i = 0; i < 6; i++) {
                        master_positions[i] = (int16_t)((self->elements.args[i * 2 + 1] << 8) | self->elements.args[i * 2]);
                    }

                    int16_t speed = 0;
                    if ((self->elements.length - 2) >= 14) {
                        speed = (int16_t)((self->elements.args[13] << 8) | self->elements.args[12]);
                    }

                    espnow_ctrl.applyMasterTeachPositions(master_positions, speed);
                }
            } break;

            case CMD_GET_REAL_JOINT_ANGLES:
            case CMD_GET_REAL_TCP_POSE:
            case CMD_SERVO_READ_OVERLOAD:
            case CMD_SERVO_WRITE_OVERLOAD:
            case CMD_SERVO_READ_BAUD:
            case CMD_SERVO_WRITE_BAUD:
            case CMD_SERVO_READ_MAX_TORQUE:
            case CMD_SERVO_WRITE_MAX_TORQUE:
            case CMD_SERVO_READ_ANGLE_LIMIT:
            case CMD_SERVO_WRITE_ANGLE_LIMIT:
            case CMD_SERVO_CALI_POS:
            case CMD_SET_COORD_LIMITS: {

                servo.tx_frame_write(0xFF, self->elements.cmd, self->elements.args, self->elements.length - 2);
                if (self->elements.length - 2 >= 24) {
                    uint8_t* a = self->elements.args;
                    PS3_X_MIN     = (float)(int16_t)((a[1]  << 8) | a[0]);
                    PS3_X_MAX     = (float)(int16_t)((a[3]  << 8) | a[2]);
                    PS3_Y_MIN     = (float)(int16_t)((a[5]  << 8) | a[4]);
                    PS3_Y_MAX     = (float)(int16_t)((a[7]  << 8) | a[6]);
                    PS3_Z_MIN     = (float)(int16_t)((a[9]  << 8) | a[8]);
                    PS3_Z_MAX     = (float)(int16_t)((a[11] << 8) | a[10]);
                    PS3_PITCH_MIN = (float)(int16_t)((a[13] << 8) | a[12]) / 10.0f;
                    PS3_PITCH_MAX = (float)(int16_t)((a[15] << 8) | a[14]) / 10.0f;
                    PS3_ROLL_MIN  = (float)(int16_t)((a[17] << 8) | a[16]);
                    PS3_ROLL_MAX  = (float)(int16_t)((a[19] << 8) | a[18]);
                    PS3_CLAW_MIN  = (float)(int16_t)((a[21] << 8) | a[20]);
                    PS3_CLAW_MAX  = (float)(int16_t)((a[23] << 8) | a[22]);
                    Serial.printf("[SYS] SET_LIMITS x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f] p[%.1f,%.1f] r[%.0f,%.0f] c[%.0f,%.0f]\n",
                                  PS3_X_MIN, PS3_X_MAX, PS3_Y_MIN, PS3_Y_MAX, PS3_Z_MIN, PS3_Z_MAX,
                                  PS3_PITCH_MIN, PS3_PITCH_MAX, PS3_ROLL_MIN, PS3_ROLL_MAX, PS3_CLAW_MIN, PS3_CLAW_MAX);
                }
                break;
            }
            case CMD_GET_COORD_LIMITS: {
                servo.tx_frame_write(0xFF, self->elements.cmd, self->elements.args, self->elements.length - 2);
                Serial.printf("[SYS] GET_LIMITS x[%.0f,%.0f] y[%.0f,%.0f] z[%.0f,%.0f] p[%.1f,%.1f] r[%.0f,%.0f] c[%.0f,%.0f]\n",
                              PS3_X_MIN, PS3_X_MAX, PS3_Y_MIN, PS3_Y_MAX, PS3_Z_MIN, PS3_Z_MAX,
                              PS3_PITCH_MIN, PS3_PITCH_MAX, PS3_ROLL_MIN, PS3_ROLL_MAX, PS3_CLAW_MIN, PS3_CLAW_MAX);
                break;
            }
            case CMD_FACTORY_RESET: {
                Serial.println("[SYS] FACTORY RESET");

                PS3_X_MIN     = 1.0f;  PS3_X_MAX     = 450.0f;
                PS3_Y_MIN     = -450.0f;  PS3_Y_MAX     = 450.0f;
                PS3_Z_MIN     =   30.0f;  PS3_Z_MAX     = 500.0f;
                PS3_PITCH_MIN =  -95.0f;  PS3_PITCH_MAX =  5.0f;
                PS3_ROLL_MIN  =  -60.0f;  PS3_ROLL_MAX  =  60.0f;
                PS3_CLAW_MIN  =  -60.0f;  PS3_CLAW_MAX  =  42.0f;

                {
                    int16_t lim[12];
                    lim[0]  = (int16_t)PS3_X_MIN;      lim[1]  = (int16_t)PS3_X_MAX;
                    lim[2]  = (int16_t)PS3_Y_MIN;      lim[3]  = (int16_t)PS3_Y_MAX;
                    lim[4]  = (int16_t)PS3_Z_MIN;      lim[5]  = (int16_t)PS3_Z_MAX;
                    lim[6]  = (int16_t)(PS3_PITCH_MIN * 10.0f);  lim[7]  = (int16_t)(PS3_PITCH_MAX * 10.0f);
                    lim[8]  = (int16_t)PS3_ROLL_MIN;    lim[9]  = (int16_t)PS3_ROLL_MAX;
                    lim[10] = (int16_t)PS3_CLAW_MIN;    lim[11] = (int16_t)PS3_CLAW_MAX;
                    servo.tx_frame_write(0xFF, CMD_SET_COORD_LIMITS, (uint8_t*)lim, 24);
                    delay(30);
                }

                {
                    uint8_t baud_args[2] = {0xFE, 0x00};  // all servos -> 1M
                    while (servo_uart_busy) { delayMicroseconds(100); }
                    servo_uart_busy = true;
                    servo.tx_frame_write(0xFE, CMD_SERVO_WRITE_BAUD, baud_args, 2);
                    servo_uart_busy = false;
                    delay(100);
                }

                for (uint8_t id = 1; id <= 6; id++) {
                    uint8_t offset_args[3] = {id, 0, 0};
                    servo.tx_frame_write(0xFF, CMD_SET_POS_OFFSET, offset_args, 3);
                    delay(30);
                }

                if (!save_servo_pid_version(0)) {
                    Serial.println("[PID] Failed to clear NVS marker before factory reset");
                }
                if (write_default_servo_configuration() && !save_servo_pid_version()) {
                    Serial.println("[PID] Factory defaults applied but NVS marker save failed");
                }

                {
                    uint8_t acc_arg = 0;

                    servo.tx_frame_write(0xFF, CMD_SET_MOVE_ACC, &acc_arg, 1);
                    delay(5);

                    uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_MOVE_ACC, &acc_arg, 1);
                    serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);

                    global_acc = 0;
                }

                arm.reset_all(2000);

                chassis_type = CHASSIS_NONE;
                chassis_wheel_diameter = 25.0f;
                chassis_wheel_base = 190.0f;
                chassis_track_width = 95.0f;
                memset(chassis_motor_dir, 0, 4);
                chassis_max_speed = 100;
                apply_chassis_config_to_runtime();

                float default_kin[9] = {110.45f, 225.00f, 36.97f, 145.00f, 0.0f, 130.23f, 0.0f, 50.0f, 70.5f};
                memcpy(arm_kin, default_kin, sizeof(arm_kin));
                servo.tx_frame_write(0xFF, CMD_SET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));

                strncpy(ps3_mac_buf, "10:00:00:00:85:95", sizeof(ps3_mac_buf));
                PS3_MAC = ps3_mac_buf;

                save_config();

                arm.board.buzzer.set(200, 100, 3, 2000);

                Serial.println("[SYS] FACTORY RESET done");
            } break;

            case CMD_SYNC_TEACH_ENTER: {
                bool can_enter = !action_edit_mode && !action_recording && !action_playback_active && !action_has_request(ACTION_REQ_ALL);
                if (can_enter) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_ENTER);
                }
                uint8_t resp[1] = {(uint8_t)(can_enter ? 1 : 0)};
                send_local_response(CMD_SYNC_TEACH_ENTER, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_EXIT: {
                sync_teach_queue_request(SYNC_TEACH_REQ_EXIT);
                uint8_t resp[1] = {1};
                send_local_response(CMD_SYNC_TEACH_EXIT, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_START: {
                bool teach_ready = sync_teach_mode || sync_teach_has_request(SYNC_TEACH_REQ_ENTER);
                bool can_start = teach_ready && !sync_teach_recording && !sync_teach_playback_active && !sync_teach_has_request(SYNC_TEACH_REQ_EXIT);
                if (can_start) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_START);
                }
                uint8_t resp[1] = {(uint8_t)(can_start ? 1 : 0)};
                send_local_response(CMD_SYNC_TEACH_START, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_STOP: {
                bool can_stop = sync_teach_recording || sync_teach_has_request(SYNC_TEACH_REQ_START);
                if (can_stop) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_STOP);
                }
                uint16_t count = sync_teach_frame_count;
                uint8_t resp[3] = {(uint8_t)(can_stop ? 1 : 0), (uint8_t)(count & 0xFF), (uint8_t)((count >> 8) & 0xFF)};
                send_local_response(CMD_SYNC_TEACH_STOP, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_PLAY: {
                bool teach_ready = sync_teach_mode || sync_teach_has_request(SYNC_TEACH_REQ_ENTER);
                bool can_play = teach_ready && !sync_teach_recording && !sync_teach_playback_active && !sync_teach_has_request(SYNC_TEACH_REQ_EXIT);
                if (can_play) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_PLAY);
                }
                uint16_t count = sync_teach_frame_count;
                uint8_t resp[3] = {(uint8_t)(can_play ? 1 : 0), (uint8_t)(count & 0xFF), (uint8_t)((count >> 8) & 0xFF)};
                send_local_response(CMD_SYNC_TEACH_PLAY, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_STOP_PLAY: {
                bool can_stop_play = sync_teach_playback_active || sync_teach_has_request(SYNC_TEACH_REQ_PLAY);
                if (can_stop_play) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_STOP_PLAY);
                }
                uint8_t resp[1] = {(uint8_t)(can_stop_play ? 1 : 0)};
                send_local_response(CMD_SYNC_TEACH_STOP_PLAY, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_CLEAR: {
                bool teach_ready = sync_teach_mode || sync_teach_has_request(SYNC_TEACH_REQ_ENTER);
                bool can_clear = teach_ready && !sync_teach_recording && !sync_teach_playback_active && !sync_teach_has_request(SYNC_TEACH_REQ_START) && !sync_teach_has_request(SYNC_TEACH_REQ_PLAY);
                if (can_clear) {
                    sync_teach_queue_request(SYNC_TEACH_REQ_CLEAR);
                }
                uint8_t resp[1] = {(uint8_t)(can_clear ? 1 : 0)};
                send_local_response(CMD_SYNC_TEACH_CLEAR, resp, sizeof(resp));
            } break;

            case CMD_SYNC_TEACH_STATUS: {
                uint16_t count = sync_teach_frame_count;
                uint8_t resp[6];
                resp[0] = sync_teach_mode ? 1 : 0;
                resp[1] = sync_teach_recording ? 1 : 0;
                resp[2] = sync_teach_playback_active ? 1 : 0;
                resp[3] = (uint8_t)(count & 0xFF);
                resp[4] = (uint8_t)((count >> 8) & 0xFF);
                resp[5] = sync_teach_overflow ? 1 : 0;
                send_local_response(CMD_SYNC_TEACH_STATUS, resp, sizeof(resp));
                Serial.printf("[SyncTeach] Status: mode=%d rec=%d play=%d frames=%u overflow=%d\n",
                              resp[0], resp[1], resp[2], count, resp[5]);
            } break;

            case CMD_ACTION_EDIT_ENTER: {
                bool can_enter = !sync_teach_blocks_action_commands() && !action_recording && !action_playback_active;
                if (can_enter) {
                    action_queue_request(ACTION_REQ_ENTER);
                }
                uint8_t resp[1] = {(uint8_t)(can_enter ? 1 : 0)};
                send_local_response(CMD_ACTION_EDIT_ENTER, resp, sizeof(resp));
            } break;

            case CMD_ACTION_EDIT_EXIT: {
                bool can_exit = action_edit_mode || action_recording || action_playback_active || action_has_request(ACTION_REQ_ENTER);
                if (can_exit) {
                    action_queue_request(ACTION_REQ_EXIT);
                }
                uint8_t resp[1] = {(uint8_t)(can_exit ? 1 : 0)};
                send_local_response(CMD_ACTION_EDIT_EXIT, resp, sizeof(resp));
            } break;

            case CMD_ACTION_EDIT_START: {
                bool edit_ready = action_edit_mode || action_has_request(ACTION_REQ_ENTER);
                bool can_start = !sync_teach_blocks_action_commands() && edit_ready && !action_recording && !action_playback_active && !action_has_request(ACTION_REQ_EXIT);
                if (can_start) {
                    action_queue_request(ACTION_REQ_START);
                    uint8_t resp[1] = {1};
                    send_local_response(CMD_ACTION_EDIT_START, resp, sizeof(resp));
                } else {
                    uint8_t resp[1] = {0};
                    send_local_response(CMD_ACTION_EDIT_START, resp, sizeof(resp));
                }
            } break;

            case CMD_ACTION_EDIT_STOP: {
                bool can_stop = action_recording || action_has_request(ACTION_REQ_START);
                if (can_stop) {
                    action_queue_request(ACTION_REQ_STOP);
                    uint16_t count = recorded_frame_count;
                    uint8_t resp[3] = {1, (uint8_t)(count & 0xFF), (uint8_t)((count >> 8) & 0xFF)};
                    send_local_response(CMD_ACTION_EDIT_STOP, resp, sizeof(resp));
                } else {
                    uint8_t resp[1] = {0};
                    send_local_response(CMD_ACTION_EDIT_STOP, resp, sizeof(resp));
                }
            } break;

            case CMD_ACTION_EDIT_PLAY: {
                bool edit_ready = action_edit_mode || action_has_request(ACTION_REQ_ENTER);
                bool can_play = !sync_teach_blocks_action_commands() && edit_ready && !action_recording && !action_playback_active && !action_has_request(ACTION_REQ_EXIT);
                if (can_play) {
                    action_queue_request(ACTION_REQ_PLAY);
                    uint16_t count = recorded_frame_count;
                    uint8_t resp[3] = {1, (uint8_t)(count & 0xFF), (uint8_t)((count >> 8) & 0xFF)};
                    send_local_response(CMD_ACTION_EDIT_PLAY, resp, sizeof(resp));
                } else {
                    uint8_t resp[1] = {0};
                    send_local_response(CMD_ACTION_EDIT_PLAY, resp, sizeof(resp));
                }
            } break;

            case CMD_ACTION_EDIT_STOP_PLAY: {
                bool can_stop_play = action_playback_active || action_has_request(ACTION_REQ_PLAY);
                if (can_stop_play) {
                    action_queue_request(ACTION_REQ_STOP_PLAY);
                    uint8_t resp[1] = {1};
                    send_local_response(CMD_ACTION_EDIT_STOP_PLAY, resp, sizeof(resp));
                } else {
                    uint8_t resp[1] = {0};
                    send_local_response(CMD_ACTION_EDIT_STOP_PLAY, resp, sizeof(resp));
                }
            } break;

            case CMD_ACTION_EDIT_CLEAR: {
                bool edit_ready = action_edit_mode || action_has_request(ACTION_REQ_ENTER);
                bool can_clear = !sync_teach_blocks_action_commands() && edit_ready && !action_recording && !action_playback_active && !action_has_request(ACTION_REQ_START) && !action_has_request(ACTION_REQ_PLAY);
                if (can_clear) {
                    action_queue_request(ACTION_REQ_CLEAR);
                    uint8_t resp[1] = {1};
                    send_local_response(CMD_ACTION_EDIT_CLEAR, resp, sizeof(resp));
                } else {
                    uint8_t resp[1] = {0};
                    send_local_response(CMD_ACTION_EDIT_CLEAR, resp, sizeof(resp));
                }
            } break;

            case CMD_ACTION_EDIT_STATUS: {
                uint16_t count = recorded_frame_count;
                uint8_t resp[5];
                resp[0] = action_edit_mode ? 1 : 0;
                resp[1] = action_recording ? 1 : 0;
                resp[2] = action_playback_active ? 1 : 0;
                resp[3] = (uint8_t)(count & 0xFF);
                resp[4] = (uint8_t)((count >> 8) & 0xFF);
                send_local_response(CMD_ACTION_EDIT_STATUS, resp, sizeof(resp));
                Serial.printf("[ActionEdit] Status: mode=%d rec=%d play=%d frames=%u\n",
                              resp[0], resp[1], resp[2], count);
                action_print_json("status", g_last_servo_positions, playback_index, count);
            } break;

            case CMD_SET_CHASSIS_CONFIG: {

                if(self->elements.length - 2 >= 1) {
                    arm.board.motor.stop();
                    chassis_type = self->elements.args[0];

                    if(self->elements.length - 2 >= 18) {
                        memcpy(&chassis_wheel_diameter, &self->elements.args[1], 4);
                        memcpy(&chassis_wheel_base, &self->elements.args[5], 4);
                        memcpy(&chassis_track_width, &self->elements.args[9], 4);
                        memcpy(chassis_motor_dir, &self->elements.args[13], 4);
                        chassis_max_speed = (int8_t)self->elements.args[17];
                    }

                    apply_chassis_config_to_runtime();
                    save_config();

                    Serial.printf("[Config] Chassis set: type=%d, wheel_d=%.1f, base=%.1f, track=%.1f, max_spd=%d\n",
                                  chassis_type, chassis_wheel_diameter, chassis_wheel_base, chassis_track_width, chassis_max_speed);

                    uint8_t resp[1] = {chassis_type};
                    len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SET_CHASSIS_CONFIG, resp, 1);
                    serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);

                    arm.board.buzzer.set(100, 100, chassis_type == CHASSIS_TANK ? 2 : 1, 2000);
                }
            } break;

            case CMD_GET_CHASSIS_CONFIG: {
                uint8_t resp[18];
                resp[0] = chassis_type;
                memcpy(&resp[1], &chassis_wheel_diameter, 4);
                memcpy(&resp[5], &chassis_wheel_base, 4);
                memcpy(&resp[9], &chassis_track_width, 4);
                memcpy(&resp[13], chassis_motor_dir, 4);
                resp[17] = (uint8_t)chassis_max_speed;

                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_GET_CHASSIS_CONFIG, resp, 18);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            } break;

            case CMD_SCAN_WIFI_CHANNELS: {

                uint8_t scan_resp[26];
                uint8_t saved_channel = global_channel;

                if (current_system_mode == MODE_ESPNOW_RUNNING) {
                    espnow_ctrl.stop();
                }

                WiFi.mode(WIFI_STA);
                WiFi.disconnect();
                delay(10);

                for (uint8_t ch = 1; ch <= 13; ch++) {
                    esp_wifi_set_promiscuous(true);
                    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
                    esp_wifi_set_promiscuous(false);

                    int n = WiFi.scanNetworks(false, true, false, 100, ch);
                    int8_t best_rssi = -127;
                    for (int i = 0; i < n; i++) {
                        if (WiFi.RSSI(i) > best_rssi) best_rssi = WiFi.RSSI(i);
                    }
                    scan_resp[(ch-1)*2]     = (uint8_t)(n > 255 ? 255 : n);
                    scan_resp[(ch-1)*2 + 1] = (uint8_t)((int8_t)best_rssi);
                    WiFi.scanDelete();
                }

                if (current_system_mode == MODE_ESPNOW_RUNNING) {
                    espnow_ctrl.begin(saved_channel);
                }

                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_SCAN_WIFI_CHANNELS, scan_resp, 26);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
                Serial.println("[SYS] WiFi channel scan complete");
            } break;

            }
    }
    else {
        if((self->elements.id >= 1 && self->elements.id <= 6) || self->elements.id == 0xFE) {

            servo.tx_frame_write(self->elements.id, self->elements.cmd, self->elements.args, self->elements.length - 2);
        }
    }
}

void system_loop_handler()
{
    service_ble_chassis_failsafe();

    if(!is_downloading) {
        serial_port.rec_handler();

        if (lerobotBridgeMode) {

            while(servo.uart->available()) {
                uint8_t c = servo.uart->read();
                at32_protocol.parsing(&c, 1);
            }

            arm.board.button.update();
            arm.board.buzzer.update();
            if(arm.board.button.is_clicked(0)) {
                ESP.restart();
            }
            return;
        }

        handle_sync_teach_requests();

        if (sync_teach_mode) {
            arm.board.button.update();
            arm.board.buzzer.update();
            pump_at32_feedback();

            if(arm.board.button.is_clicked(0)) {
                esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_BOOT_BUTTON, NULL, 0, 0);
                uint8_t btn_data[2] = {0, 1};
                uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            }
            if(arm.board.button.is_clicked(1)) {
                esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_USER_BUTTON, NULL, 0, 0);
                uint8_t btn_data[2] = {1, 1};
                uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            }
            return;
        }

        handle_action_edit_requests();
        handle_action_recording();

        if(current_system_mode == MODE_WIFI_AP_RUNNING) {
            espnow_ctrl.server_loop();
        }

        if (action_edit_mode) {
            arm.board.button.update();
            arm.board.buzzer.update();
            pump_at32_feedback();

            if(arm.board.button.is_clicked(0)) {
                esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_BOOT_BUTTON, NULL, 0, 0);
                uint8_t btn_data[2] = {0, 1};
                uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            }
            if(arm.board.button.is_clicked(1)) {
                esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_USER_BUTTON, NULL, 0, 0);
                uint8_t btn_data[2] = {1, 1};
                uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            }
            return;
        }

        ps3_repeat_handler();
        colorGrabber.update();
        colorTrackerRot.update();
        faceTracker.update();
        selfLearningTracker.update();
        arm.board.button.update();
        aprilTagTracker.update();
        aprilTagGrabber.update();
        if (!lerobotBridgeMode) aiLLMControl.update();
        garbageGrabber.update();
        gestureTracker.update();
        calibrationGrabber.update();

        while(servo.uart->available()) {
            uint8_t c = servo.uart->read();
            at32_protocol.parsing(&c, 1);
        }

        if(arm.board.button.is_clicked(0)) {
            esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_BOOT_BUTTON, NULL, 0, 0);
            uint8_t btn_data[2] = {0, 1};
            uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
        }
        if(arm.board.button.is_clicked(1)) {
            esp_event_post_to(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_USER_BUTTON, NULL, 0, 0);
            uint8_t btn_data[2] = {1, 1};
            uint8_t len = serial_port.protocol.tx_packet_complete(0xFF, CMD_BUTTON_EVENT, btn_data, 2);
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
        }
    } else {
        serial_port.rec_handler();
    }
}

void register_system_task(esp_event_loop_handle_t *event_loop)
{
    loop_with_sys_task = *event_loop;
    setCpuFrequencyMhz(240);
    load_config();

    if (!LittleFS.begin(true)) {
        Serial.println("[LittleFS] Mount failed, formatting...");
        LittleFS.format();
        LittleFS.begin();
    }
    Serial.println("[LittleFS] Mounted successfully");

    serial_port.begin(Serial, 1000000);
    serial_port.register_ops_callback(non_ble_ctrl_callback);

    servo.begin(Serial1, 1000000, 16, 17);
    arm.begin();

    arm.board.oled.set_icon(4);
    for (int i = 0; i < 25; i++) {
        arm.board.oled.show_icon();
        delay(100);
    }

    AT32_OTA::check_and_update(serial_port.protocol, *servo.uart, arm.board.oled);
    initialize_servo_pid_once();

    servo.tx_frame_write(0xFF, CMD_SET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));
    delay(50);
    save_config();
    arm.set_torque(true);
    delay(100);
    arm.update_status();
    colorTrackerRot.begin();
    faceTracker.begin();
    selfLearningTracker.begin();
    aprilTagTracker.begin();
    aprilTagGrabber.begin();
    at32_protocol.begin();
    aiLLMControl.begin();
    colorGrabber.begin();
    garbageGrabber.begin();
    gestureTracker.begin();
    calibrationGrabber.begin();
    at32_protocol.register_success_callback(at32_packet_callback);

    current_system_mode = MODE_WIFI_AP_RUNNING;
    espnow_ctrl.startAP(AP_SSID, AP_PASS);
    espnow_ctrl.register_ops_callback(non_ble_ctrl_callback);

    Serial.printf("[BT] current_bt_mode=%d (%s)\n", current_bt_mode,
                  current_bt_mode == BT_MODE_PS3 ? "PS3" : "BLE");
    if(current_bt_mode == BT_MODE_BLE) {
        Serial.println("Initializing BLE mode...");
        ble_port.register_connection_callback(ble_connection_callback);
        ble_port.begin("Nex_Arm");
        ble_port.register_ops_callback(ble_ctrl_callback);
    } else if(current_bt_mode == BT_MODE_PS3) {
        Serial.println("Initializing PS3 Controller mode...");
        ps3_port.begin(PS3_MAC);
        ps3_port.register_button_callback(ps3_button_callback);
        ps3_port.register_analog_callback(ps3_analog_callback);
    }

    arm.board.oled.show_wifi_ap_info(String(AP_SSID), WiFi.softAPIP());

    for (int i = 0; i < 10; i++) {
        arm.update_status();
        delay(50);
        while(servo.uart->available()) {
            uint8_t c = servo.uart->read();
            at32_protocol.parsing(&c, 1);
        }
    }

    ps3_target_x = arm.current_pose.x;
    ps3_target_y = arm.current_pose.y;
    ps3_target_z = arm.current_pose.z;
    ps3_target_pitch = arm.current_pose.pitch;
    ps3_target_roll = arm.current_pose.roll;
    ps3_target_claw = arm.current_pose.claw;
    Serial.printf("[PS3 Init] synced pose x=%.1f y=%.1f z=%.1f p=%.1f r=%.1f c=%.1f\n",
                  ps3_target_x, ps3_target_y, ps3_target_z,
                  ps3_target_pitch, ps3_target_roll, ps3_target_claw);

    esp_event_handler_instance_register_with(loop_with_sys_task, SYS_TIMING_EVENTS, ESP_EVENT_ANY_ID, sys_timer_sub_handler, NULL, NULL);
    esp_event_handler_instance_register_with(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_BOOT_BUTTON, boot_button_sub_handler, NULL, NULL);
    esp_event_handler_instance_register_with(loop_with_sys_task, SYS_STATUS_EVENTS, STATUS_EVENT_USER_BUTTON, user_button_sub_handler, NULL, NULL);

    TIMER = xTimerCreate("sys_timing", pdMS_TO_TICKS(TIMER_PERIOD), pdTRUE, NULL, sys_timer_post_callback);
    xTimerStart(TIMER, 0);
}
