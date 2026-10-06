#include "system_task_handle.h"
#include "Global.h"
#include "Robot_Arm.h"
#include "usb_ctrl.h"
#include <Preferences.h>
#include "ColorGrabber.h"
#include "AT32_OTA.h"
#include <LittleFS.h>
#include <math.h>
#include <string.h>

#define CMD_SERVO_STREAM 0x68

static const char* TAG = "ai_vision_sys";
static TimerHandle_t TIMER;
static esp_event_loop_handle_t loop_with_sys_task;
static CommProtocol_t at32_protocol;
static Preferences prefs;
static bool is_downloading = false;

static int16_t g_last_servo_positions[6] = {2048, 2048, 2048, 2048, 2048, 2048};
static uint32_t g_last_servo_feedback_ms = 0;

static const uint8_t SERVO_COUNT = 6;
static const int16_t POSE_STABLE_SPAN = 18;
static const int16_t POSE_LOCK_VERIFY_DELTA = 35;
static const int16_t HOME_RESTORE_MAX_DELTA = 350;
static const int16_t HOME_RESTORE_SPEED = 180;
static bool s_pose_calibration_active = false;
static bool s_home_pose_valid = false;
static int16_t s_home_servo_positions[SERVO_COUNT] = {2048, 2048, 2048, 2048, 2048, 2048};

uint8_t global_channel = 2;
uint8_t global_acc = 245;

static float arm_kin[9] = {
    110.45f, 225.00f, 36.97f, 145.00f, 0.0f, 130.23f, 0.0f, 50.0f, 70.5f
};

ESP_EVENT_DEFINE_BASE(SYS_TIMING_EVENTS);

/* KEY1 = User(GPIO2) id 1 选色；KEY2 = Boot(GPIO0) id 0 启动分拣（本版不按 Boot 重启） */
static const char* kProtoColors[] = {"red", "green", "blue"};
static const char* kOledColor[]  = {"RED", "GREEN", "BLUE"};

static uint8_t  s_color_sel_idx = 0;
static bool     s_key1_used_since_idle = false;
static bool     s_was_grabber_busy = false;

static void apply_oled_standby_screen(void)
{
    arm.board.oled.set_custom_text(0, "Color Sorting");
    if (s_pose_calibration_active) {
        arm.board.oled.set_custom_text(1, "Torque: OFF");
        arm.board.oled.set_custom_text(2, "Adjust all joints");
        arm.board.oled.set_custom_text(3, "KEY1: save+lock");
    } else if (colorGrabber.isBusy()) {
        arm.board.oled.set_custom_text(1, "Running...");
        arm.board.oled.set_custom_text(2, "");
        arm.board.oled.set_custom_text(3, "Please wait");
    } else {
        if (s_key1_used_since_idle) {
            arm.board.oled.set_custom_text(1, "KEY1: Run this");
        } else {
            arm.board.oled.set_custom_text(1, "KEY1: Run(def RED)");
        }
        String line2 = "KEY2:";
        line2 += kOledColor[s_color_sel_idx % 3];
        arm.board.oled.set_custom_text(2, line2);
        arm.board.oled.set_custom_text(3, "Idle");
    }
    arm.board.oled.show_custom();
}

static void on_key1_select_color(void)
{
    if (colorGrabber.isBusy()) return;
    s_color_sel_idx = (uint8_t)((s_color_sel_idx + 1) % 3);
    s_key1_used_since_idle = true;
    arm.board.buzzer.set(30, 30, 1, 2500);
    apply_oled_standby_screen();
    Serial.printf("[ColorSort] KEY1 -> next color %s\n", kProtoColors[s_color_sel_idx % 3]);
}

static void on_key2_start_sorting(void)
{
    if (colorGrabber.isBusy()) {
        Serial.println("[ColorSort] Busy, ignore KEY2");
        return;
    }
    const char* cname = "red";
    if (s_key1_used_since_idle) {
        cname = kProtoColors[s_color_sel_idx % 3];
    }
    const char* colors[] = {cname};
    colorGrabber.start(colors, 1);
    arm.board.buzzer.set(100, 100, 2, 2000);
    apply_oled_standby_screen();
    Serial.printf("[ColorSort] KEY2 -> start '%s' (key1_used=%d)\n", cname, (int)s_key1_used_since_idle);
}

void save_config(void)
{
    prefs.begin("robot_cfg", false);
    prefs.putUChar("channel", global_channel);
    prefs.putUChar("acc", global_acc);
    prefs.putBytes("arm_kin", arm_kin, sizeof(arm_kin));
    prefs.putFloat("grab_z_adj", colorGrabber.getGrabZAdjust());
    if (s_home_pose_valid) {
        prefs.putBytes("home_pos", s_home_servo_positions, sizeof(s_home_servo_positions));
        prefs.putBool("home_valid", true);
    }
    prefs.end();
}

void load_config(void)
{
    prefs.begin("robot_cfg", true);
    global_channel = prefs.getUChar("channel", 2);
    global_acc = prefs.getUChar("acc", 245);

    float ag_x = prefs.getFloat("ag_x", 50.0f);
    float ag_y = prefs.getFloat("ag_y", 15.0f);
    float ag_z = prefs.getFloat("ag_z", 60.0f);
    colorGrabber.setOffsets(ag_x, ag_y, ag_z);
    colorGrabber.setGrabZAdjust(prefs.getFloat("grab_z_adj", -12.0f));

    if (prefs.isKey("arm_kin")) {
        float saved_kin[9];
        prefs.getBytes("arm_kin", saved_kin, sizeof(saved_kin));
        if (fabsf(saved_kin[0] - arm_kin[0]) < 0.01f) {
            memcpy(arm_kin, saved_kin, sizeof(arm_kin));
        }
    }

    s_home_pose_valid = false;
    if (prefs.getBool("home_valid", false) &&
        prefs.getBytesLength("home_pos") == sizeof(s_home_servo_positions)) {
        size_t read_len = prefs.getBytes("home_pos", s_home_servo_positions,
                                         sizeof(s_home_servo_positions));
        s_home_pose_valid = (read_len == sizeof(s_home_servo_positions));
        for (uint8_t i = 0; i < SERVO_COUNT && s_home_pose_valid; ++i) {
            if (s_home_servo_positions[i] < 0 || s_home_servo_positions[i] > 4095) {
                s_home_pose_valid = false;
            }
        }
    }
    prefs.end();
}

static bool request_fresh_servo_positions(int16_t out_pos[SERVO_COUNT], uint32_t timeout_ms)
{
    uint32_t previous_feedback_ms = g_last_servo_feedback_ms;
    arm.update_status();
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        serial_port.rec_handler();
        pump_at32_feedback();
        if (g_last_servo_feedback_ms != 0 &&
            g_last_servo_feedback_ms != previous_feedback_ms) {
            memcpy(out_pos, g_last_servo_positions, sizeof(g_last_servo_positions));
            return true;
        }
        delay(2);
    }
    return false;
}

static bool capture_stable_servo_pose(int16_t out_pos[SERVO_COUNT])
{
    const uint8_t sample_count = 5;
    int16_t min_pos[SERVO_COUNT];
    int16_t max_pos[SERVO_COUNT];
    int32_t sum_pos[SERVO_COUNT] = {0, 0, 0, 0, 0, 0};

    for (uint8_t sample = 0; sample < sample_count; ++sample) {
        int16_t current[SERVO_COUNT];
        if (!request_fresh_servo_positions(current, 350)) {
            return false;
        }

        for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
            if (current[i] < 0 || current[i] > 4095) {
                return false;
            }
            if (sample == 0) {
                min_pos[i] = current[i];
                max_pos[i] = current[i];
            } else {
                if (current[i] < min_pos[i]) min_pos[i] = current[i];
                if (current[i] > max_pos[i]) max_pos[i] = current[i];
            }
            sum_pos[i] += current[i];
        }
        delay(40);
    }

    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        if (max_pos[i] - min_pos[i] > POSE_STABLE_SPAN) {
            return false;
        }
        out_pos[i] = (int16_t)(sum_pos[i] / sample_count);
    }
    return true;
}

static void write_all_servo_targets(const int16_t positions[SERVO_COUNT], int16_t speed)
{
    uint8_t ids[SERVO_COUNT] = {1, 2, 3, 4, 5, 6};
    int16_t targets[SERVO_COUNT];
    memcpy(targets, positions, sizeof(targets));
    servo.sync_write_pos_speed(ids, targets, speed, SERVO_COUNT);
}

static void enter_pose_calibration(const char* reason)
{
    arm.set_torque(false);
    delay(80);
    s_pose_calibration_active = true;
    apply_oled_standby_screen();
    Serial.printf("[PoseCal] Torque OFF: %s\n", reason ? reason : "manual calibration");
    Serial.println("[PoseCal] Support the arm, adjust all joints, then press KEY1");
}

static bool verify_pose_near(const int16_t target[SERVO_COUNT], int16_t max_delta)
{
    int16_t actual[SERVO_COUNT];
    if (!request_fresh_servo_positions(actual, 500)) return false;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        if (abs((int32_t)actual[i] - (int32_t)target[i]) > max_delta) {
            return false;
        }
    }
    return true;
}

static bool save_and_lock_manual_pose(void)
{
    arm.board.oled.set_custom_text(0, "Pose calibration");
    arm.board.oled.set_custom_text(1, "Release KEY1");
    arm.board.oled.set_custom_text(2, "Hold arm still");
    arm.board.oled.set_custom_text(3, "Reading...");
    arm.board.oled.show_custom();

    uint32_t settle_start = millis();
    while (millis() - settle_start < 700) {
        pump_at32_feedback();
        arm.board.button.update();
        delay(5);
    }

    int16_t captured[SERVO_COUNT];
    if (!capture_stable_servo_pose(captured)) {
        arm.board.oled.set_custom_text(1, "Read unstable");
        arm.board.oled.set_custom_text(2, "Hold arm still");
        arm.board.oled.set_custom_text(3, "KEY1: retry");
        arm.board.oled.show_custom();
        arm.board.buzzer.set(180, 80, 2, 900);
        Serial.println("[PoseCal] Capture failed or unstable; torque remains OFF");
        return false;
    }

    // Set the goal registers before enabling torque so no servo returns to an old goal.
    write_all_servo_targets(captured, 80);
    delay(100);
    arm.set_torque(true);
    delay(300);

    if (!verify_pose_near(captured, POSE_LOCK_VERIFY_DELTA)) {
        arm.set_torque(false);
        arm.board.oled.set_custom_text(1, "Lock check failed");
        arm.board.oled.set_custom_text(2, "Torque remains OFF");
        arm.board.oled.set_custom_text(3, "KEY1: retry");
        arm.board.oled.show_custom();
        arm.board.buzzer.set(200, 100, 3, 700);
        Serial.println("[PoseCal] Lock verification failed; torque switched OFF");
        return false;
    }

    memcpy(s_home_servo_positions, captured, sizeof(s_home_servo_positions));
    s_home_pose_valid = true;
    save_config();
    s_pose_calibration_active = false;
    arm.board.buzzer.set(80, 60, 2, 2600);
    Serial.printf("[PoseCal] Saved+locked: %d,%d,%d,%d,%d,%d\n",
                  captured[0], captured[1], captured[2],
                  captured[3], captured[4], captured[5]);
    apply_oled_standby_screen();
    return true;
}

static bool restore_saved_home_pose(void)
{
    if (!s_home_pose_valid) return false;

    int16_t actual[SERVO_COUNT];
    if (!request_fresh_servo_positions(actual, 600)) {
        Serial.println("[PoseCal] Cannot read startup pose");
        return false;
    }

    int32_t max_delta = 0;
    for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
        int32_t delta = abs((int32_t)actual[i] - (int32_t)s_home_servo_positions[i]);
        if (delta > max_delta) max_delta = delta;
    }
    if (max_delta > HOME_RESTORE_MAX_DELTA) {
        Serial.printf("[PoseCal] Startup pose too far from saved home: %ld ticks\n", (long)max_delta);
        return false;
    }

    write_all_servo_targets(s_home_servo_positions, HOME_RESTORE_SPEED);
    delay(100);
    arm.set_torque(true);

    uint32_t wait_ms = (uint32_t)(max_delta * 1000L / HOME_RESTORE_SPEED) + 800;
    if (wait_ms > 5000) wait_ms = 5000;
    uint32_t start = millis();
    while (millis() - start < wait_ms) {
        pump_at32_feedback();
        delay(10);
    }

    if (!verify_pose_near(s_home_servo_positions, POSE_LOCK_VERIFY_DELTA)) {
        arm.set_torque(false);
        Serial.println("[PoseCal] Saved-home verification failed; torque switched OFF");
        return false;
    }

    Serial.printf("[PoseCal] Restored saved home: %d,%d,%d,%d,%d,%d\n",
                  s_home_servo_positions[0], s_home_servo_positions[1],
                  s_home_servo_positions[2], s_home_servo_positions[3],
                  s_home_servo_positions[4], s_home_servo_positions[5]);
    return true;
}

static void sys_timer_post_callback(TimerHandle_t xTimer)
{
    if (is_downloading) return;

    static uint32_t count = 0;
    if (count % BUZZER_UPDATE_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_BUZZER_UPDATE, NULL, 0, 0);
    }
    if (count % BAT_UPDATE_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_BAT_UPDATE, NULL, 0, 0);
    }
    if (count % SERVO_STATUS_UPDATE_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_SERVO_STATUS_UPDATE, NULL, 0, 0);
    }
    if (count % OLED_REFRESH_PERIOD == 0) {
        esp_event_post_to(loop_with_sys_task, SYS_TIMING_EVENTS, TIMING_EVENT_OLED_REFRESH, NULL, 0, 0);
    }

    count += TIMER_PERIOD;
}

static void sys_timer_sub_handler(void* handler_args, esp_event_base_t base, int32_t id, void* event_data)
{
    if (is_downloading) return;

    switch (id) {
        case TIMING_EVENT_BUZZER_UPDATE:
            arm.board.buzzer.update();
            break;
        case TIMING_EVENT_BAT_UPDATE:
            arm.board.bat.update();
            break;
        case TIMING_EVENT_SERVO_STATUS_UPDATE:
            arm.update_status();
            break;
        case TIMING_EVENT_OLED_REFRESH:
            apply_oled_standby_screen();
            break;
        default:
            break;
    }
}

static void func_ctrl_callback(PacketTypeDef* self)
{
    uint8_t len;
    uint16_t bat_level;

    if (self->elements.id == 0xFF && self->elements.cmd == CMD_ACTION_GROUP_DOWNLOAD) {
        is_downloading = true;
        arm.board.action_group_download(self->elements.args[0], self->elements.args, self->elements.length - 2);
        is_downloading = false;
        return;
    }

    if (self->elements.id != 0xFF) return;

    switch (self->elements.cmd) {
        case CMD_FIRMWARE_VERSION_CHECK: {
            uint8_t at32_ver[3] = {0, 0, 0};
            AT32_OTA::query_version(at32_protocol, *servo.uart, at32_ver);
            uint8_t ver[6] = {
                FIRMWARE_VERSION_MAJOR, FIRMWARE_VERSION_MINOR, FIRMWARE_VERSION_PATCH,
                at32_ver[0], at32_ver[1], at32_ver[2]
            };
            len = serial_port.protocol.tx_packet_complete(0xFF, CMD_FIRMWARE_VERSION_CHECK, ver, 6);
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            Serial.printf("[FW] ESP32=%d.%d.%d AT32=%d.%d.%d\n",
                          ver[0], ver[1], ver[2], ver[3], ver[4], ver[5]);
        } break;

        case CMD_CHECK_BAT_LEVEL_CHECK:
            bat_level = (uint16_t)arm.board.bat.get_voltage();
            len = serial_port.protocol.tx_packet_complete(0xFF, CMD_CHECK_BAT_LEVEL_CHECK, (uint8_t*)&bat_level, sizeof(bat_level));
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
            break;

        case CMD_SET_KINEMATICS_PARAM:
            if (self->elements.length - 2 >= (int)sizeof(arm_kin)) {
                memcpy(arm_kin, self->elements.args, sizeof(arm_kin));
                servo.tx_frame_write(0xFF, CMD_SET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));
                save_config();
                arm.board.buzzer.set(100, 50, 1, 3000);
            }
            break;

        case CMD_APRILTAG_SET_OFFSET:
            if (self->elements.length >= 14) {
                float x_off, y_off, z_grab;
                memcpy(&x_off, &self->elements.args[0], 4);
                memcpy(&y_off, &self->elements.args[4], 4);
                memcpy(&z_grab, &self->elements.args[8], 4);
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
            if (self->elements.length >= 4) {
                uint8_t sw = self->elements.args[0];
                uint8_t color_id = self->elements.args[1];
                if (sw == 0) {
                    colorGrabber.stop();
                    arm.board.buzzer.off();
                } else {
                    const char* cName = "";
                    if (color_id == 1) cName = "red";
                    else if (color_id == 2) cName = "green";
                    else if (color_id == 3) cName = "blue";
                    if (strlen(cName) > 0) {
                        colorGrabber.stop();
                        delay(100);
                        const char* colors[] = {cName};
                        colorGrabber.start(colors, 1);
                        arm.board.buzzer.set(100, 100, 2, 2000);
                    }
                }
            }
            break;

        default:
            break;
    }
}

void at32_packet_callback(PacketTypeDef* rx_packet)
{
    if (is_downloading) return;

    if (rx_packet->elements.id == 0xFF &&
        rx_packet->elements.cmd == CMD_SERVO_STREAM &&
        rx_packet->elements.length >= 14) {
        for (int i = 0; i < 6; ++i) {
            int idx = i * 2;
            g_last_servo_positions[i] = (int16_t)((rx_packet->elements.args[idx + 1] << 8) | rx_packet->elements.args[idx]);
        }
        g_last_servo_feedback_ms = millis();
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
        }
    }
}

void pump_at32_feedback(void)
{
    while (servo.uart->available()) {
        uint8_t c = servo.uart->read();
        at32_protocol.parsing(&c, 1);
    }
}

bool sync_arm_feedback(uint32_t timeout_ms)
{
    arm.update_status();
    uint32_t start = millis();
    uint32_t prev_feedback_ms = g_last_servo_feedback_ms;

    while (millis() - start < timeout_ms) {
        serial_port.rec_handler();
        pump_at32_feedback();
        if (g_last_servo_feedback_ms != 0 && g_last_servo_feedback_ms != prev_feedback_ms) {
            return true;
        }
        delay(2);
    }

    return g_last_servo_feedback_ms != 0;
}

bool get_last_servo_positions(int16_t out_pos[6])
{
    if (g_last_servo_feedback_ms == 0) {
        return false;
    }
    memcpy(out_pos, g_last_servo_positions, sizeof(g_last_servo_positions));
    return true;
}

bool sync_teach_handle_master_packet(uint32_t seq, const int16_t master_positions[6])
{
    (void)seq;
    (void)master_positions;
    return false;
}

void system_loop_handler(void)
{
    if (!is_downloading) {
        serial_port.rec_handler();
        pump_at32_feedback();

        arm.board.button.update();
        arm.board.buzzer.update();

        if (s_pose_calibration_active) {
            if (arm.board.button.is_clicked(1)) {
                save_and_lock_manual_pose();
            }
            return;
        }

        colorGrabber.update();

        bool busy_now = colorGrabber.isBusy();
        if (s_was_grabber_busy && !busy_now) {
            s_key1_used_since_idle = false;
            Serial.println("[ColorSort] Run finished, idle (KEY1 pick / KEY2 start)");
        }
        s_was_grabber_busy = busy_now;

        if (!busy_now) {
            if (arm.board.button.is_long_clicked(1)) {
                enter_pose_calibration("KEY1 long press");
                return;
            }
            if (arm.board.button.is_clicked(1)) {
                on_key1_select_color();
            }
            if (arm.board.button.is_clicked(0)) {
                on_key2_start_sorting();
            }
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
    serial_port.register_ops_callback(func_ctrl_callback);

    servo.begin(Serial1, 1000000, 16, 17);
    arm.begin();

    arm.board.oled.set_icon(4);
    for (int i = 0; i < 25; i++) {
        arm.board.oled.show_icon();
        delay(100);
    }

    AT32_OTA::check_and_update(serial_port.protocol, *servo.uart, arm.board.oled);

    servo.tx_frame_write(0xFF, CMD_SET_KINEMATICS_PARAM, (uint8_t*)arm_kin, sizeof(arm_kin));
    delay(50);

    at32_protocol.begin();
    at32_protocol.register_success_callback(at32_packet_callback);

    colorGrabber.begin();

    // Always start with all six servos released. Goals are written before torque is enabled.
    arm.set_torque(false);
    delay(100);

    if (!restore_saved_home_pose()) {
        enter_pose_calibration(s_home_pose_valid ? "startup restore failed" : "no saved home");
    } else {
        apply_oled_standby_screen();
    }

    esp_event_handler_instance_register_with(loop_with_sys_task, SYS_TIMING_EVENTS, ESP_EVENT_ANY_ID,
                                             sys_timer_sub_handler, NULL, NULL);

    TIMER = xTimerCreate("sys_timing", pdMS_TO_TICKS(TIMER_PERIOD), pdTRUE, NULL, sys_timer_post_callback);
    xTimerStart(TIMER, 0);

    Serial.printf("[%s] Color sort firmware %s ready\n", TAG, FIRMWARE_VERSION_STR);
}
