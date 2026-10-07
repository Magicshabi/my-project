#include "system_task_handle.h"
#include "Global.h"
#include "Robot_Arm.h"
#include "usb_ctrl.h"
#include <Preferences.h>
#include "ColorGrabber.h"
#include "AT32_OTA.h"
#include "ProtocolSelfTest.h"
#include "HandleTeachRecord.h"
#include "RealJointFeedback.h"
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
static int16_t g_real_joint_positions[6] = {};
static uint32_t g_real_joint_sequence = 0;

static const uint8_t SERVO_COUNT = 6;
static const int16_t POSE_STABLE_SPAN = 8;
static const int16_t POSE_LOCK_VERIFY_DELTA = 8;
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

/* Official KeyScan_Beep mapping: KEY1 = BOOT/GPIO0, KEY2 = USER/GPIO2. */
static constexpr uint8_t KEY1_BUTTON_ID = 0;
static constexpr uint8_t KEY2_BUTTON_ID = 1;
/* 赛题要求红/蓝/黄/绿四色。注意：K230 侧是否认 "yellow" 必须实机验证，
 * 若认不出会走坐标兜底或第 4 色替换策略，此处先按 4 色开放按键循环。 */
#define KEY_COLOR_COUNT 4
static const char* kProtoColors[] = {"red", "green", "blue", "yellow"};
static const char* kOledColor[]  = {"RED", "GREEN", "BLUE", "YELLOW"};

static uint8_t  s_color_sel_idx = 0;
static bool     s_key1_used_since_idle = false;
static bool     s_was_grabber_busy = false;

// Dedicated teaching build: no automatic picking, including serial commands.
enum HandleTeachState { HT_IDLE, HT_CONFIRM, HT_RELEASED, HT_HELD, HT_SAVE_FAILED, HT_DONE };
static HandleTeachState s_handle_state = HT_IDLE;
static HandleTeachRecord s_handle_record = new_handle_record();
static uint8_t s_handle_slot = 0;
static const char* s_handle_error = nullptr;
static const char* const kHandleStages[8] = {
    "1 Red above OPEN", "2 Red down OPEN", "3 Red down GRIP", "4 Red lift GRIP",
    "5 T0 above GRIP", "6 T0 down GRIP", "7 T0 down OPEN", "8 T0 clear OPEN"
};
static void handle_teach_keys(bool key1, bool key2);
static void load_handle_teach();

static void show_handle_teach() {
    arm.board.oled.set_custom_text(0, kHandleStages[s_handle_slot]);
    if (s_handle_state == HT_CONFIRM) {
        arm.board.oled.set_custom_text(1, "Support arm first");
        arm.board.oled.set_custom_text(2, "KEY1: torque OFF");
        arm.board.oled.set_custom_text(3, "KEY2: cancel");
    } else if (s_handle_state == HT_RELEASED) {
        arm.board.oled.set_custom_text(1, s_handle_error ? s_handle_error : "Torque OFF: adjust");
        arm.board.oled.set_custom_text(2, "KEY1: save+lock");
        arm.board.oled.set_custom_text(3, "Keep arm supported");
    } else if (s_handle_state == HT_SAVE_FAILED) {
        arm.board.oled.set_custom_text(1, "Held; save FAILED");
        arm.board.oled.set_custom_text(2, "KEY1: retry save");
        arm.board.oled.set_custom_text(3, "KEY2: redo prep");
    } else if (s_handle_state == HT_HELD) {
        arm.board.oled.set_custom_text(1, "Saved + held");
        arm.board.oled.set_custom_text(2, "KEY1: redo prep");
        arm.board.oled.set_custom_text(3, s_handle_slot == 7 ? "KEY2: finish" : "KEY2: next prep");
    } else {
        arm.board.oled.set_custom_text(0, "RED -> T0 saved");
        arm.board.oled.set_custom_text(1, "8 poses; held");
        arm.board.oled.set_custom_text(2, "Playback disabled");
        arm.board.oled.set_custom_text(3, "KEY2: exit");
    }
    arm.board.oled.show_custom();
}

static void apply_oled_standby_screen(void)
{
    if (!s_pose_calibration_active && s_handle_state != HT_IDLE) {
        show_handle_teach();
        return;
    }
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
        arm.board.oled.set_custom_text(0, "RED handle teach");
        arm.board.oled.set_custom_text(1, "KEY1: teach prep");
        arm.board.oled.set_custom_text(2, "KEY2: NO RUN");
        arm.board.oled.set_custom_text(3, s_handle_record.valid_mask == 255 ? "8 poses saved" : "Teach required");
    }
    arm.board.oled.show_custom();
}

static void on_key1_select_color(void)
{
    if (colorGrabber.isBusy()) return;
    s_color_sel_idx = (uint8_t)((s_color_sel_idx + 1) % KEY_COLOR_COUNT);
    s_key1_used_since_idle = true;
    arm.board.buzzer.set(30, 30, 1, 2500);
    apply_oled_standby_screen();
    Serial.printf("[ColorSort] KEY1 -> next color %s\n", kProtoColors[s_color_sel_idx % KEY_COLOR_COUNT]);
}

static void on_key2_start_sorting(void)
{
    if (colorGrabber.isBusy()) {
        Serial.println("[ColorSort] Busy, ignore KEY2");
        return;
    }
    const char* cname = "red";
    if (s_key1_used_since_idle) {
        cname = kProtoColors[s_color_sel_idx % KEY_COLOR_COUNT];
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

/* ── 放置槽位坐标表的掉电保存 ──
 * 必须放在 colorGrabber.begin() 之后调用，否则会被 begin() 里的默认值覆盖。
 * 有了它，标定只需串口下发一次，不用每次改坐标都重新烧固件。 */
static void save_place_slots(void)
{
    prefs.begin("robot_cfg", false);
    prefs.putBytes("place_tab", (const uint8_t*)colorGrabber.placeTableRaw(),
                   colorGrabber.placeTableBytes());
    prefs.end();
}

static void load_place_slots(void)
{
    prefs.begin("robot_cfg", true);
    size_t n = prefs.getBytesLength("place_tab");
    if (n == (size_t)colorGrabber.placeTableBytes()) {
        prefs.getBytes("place_tab", (void*)colorGrabber.placeTableRaw(), n);
        Serial.println("[ColorSort] place slots loaded from NVS");
    } else {
        Serial.println("[ColorSort] no saved place slots -> using defaults");
    }
    prefs.end();

    for (uint8_t s = 0; s < ColorGrabber::PLACE_SLOTS; s++) {
        float x = 0, y = 0, z = 0;
        colorGrabber.getPlaceSlot(s, &x, &y, &z);
        Serial.printf("[ColorSort]   slot %d = (%.1f, %.1f, %.1f)\n", s, x, y, z);
    }
}

/* ── 抓取源坐标表的掉电保存 ──
 * 同样必须在 colorGrabber.begin() 之后调用。
 * 「是否已示教」(pick_val) 也一起存，未示教的颜色不会触发视觉降级。 */
static void save_pick_table(void)
{
    prefs.begin("robot_cfg", false);
    prefs.putBytes("pick_tab", (const uint8_t*)colorGrabber.pickTableRaw(),
                   colorGrabber.pickTableBytes());
    prefs.putBytes("pick_val", colorGrabber.pickValidRaw(),
                   colorGrabber.pickValidBytes());
    prefs.putUChar("pick_mode", colorGrabber.getPickMode());
    prefs.end();
}

static void load_pick_table(void)
{
    static const char* kPickNames[] = {"red", "green", "blue", "yellow"};

    prefs.begin("robot_cfg", true);
    size_t n1 = prefs.getBytesLength("pick_tab");
    size_t n2 = prefs.getBytesLength("pick_val");
    if (n1 == (size_t)colorGrabber.pickTableBytes() &&
        n2 == (size_t)colorGrabber.pickValidBytes()) {
        prefs.getBytes("pick_tab", (void*)colorGrabber.pickTableRaw(), n1);
        prefs.getBytes("pick_val", (void*)colorGrabber.pickValidRaw(), n2);
        Serial.println("[ColorSort] pick table loaded from NVS");
    } else {
        Serial.println("[ColorSort] no saved pick table (纯视觉，不降级)");
    }
    colorGrabber.setPickMode(prefs.getUChar("pick_mode", 0));
    prefs.end();

    for (uint8_t i = 0; i < ColorGrabber::PICK_COLORS; i++) {
        float x = 0, y = 0, z = 0;
        colorGrabber.getPickSlot(i, &x, &y, &z);
        Serial.printf("[ColorSort]   pick %-6s = (%.1f, %.1f, %.1f) %s\n",
                      kPickNames[i], x, y, z,
                      colorGrabber.isPickValid(i) ? "" : "<- 未示教");
    }
    Serial.printf("[ColorSort] pickmode = %d (0=视觉优先 1=强制坐标 2=强制视觉)\n",
                  colorGrabber.getPickMode());
}

static bool request_fresh_servo_positions(int16_t out_pos[SERVO_COUNT], uint32_t timeout_ms)
{
    // Drain queued packets before baseline. No command-11/stream fallback.
    pump_at32_feedback();
    uint32_t previous_sequence = g_real_joint_sequence;
    servo.tx_frame_write(0xFF, CMD_GET_REAL_JOINT_ANGLES, nullptr, 0);
    servo.uart->flush();
    uint32_t start = millis();

    while (millis() - start < timeout_ms) {
        serial_port.rec_handler();
        pump_at32_feedback();
        if (g_real_joint_sequence != previous_sequence) {
            memcpy(out_pos, g_real_joint_positions, sizeof(g_real_joint_positions));
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
        if (!request_fresh_servo_positions(current, 350)) return false;

        for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
            if (current[i] < 0 || current[i] > 4095) return false;
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
        if (max_pos[i] - min_pos[i] > POSE_STABLE_SPAN) return false;
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
        if (abs((int32_t)actual[i] - (int32_t)target[i]) > max_delta) return false;
    }
    return true;
}

static bool lock_at_measured_pose(const int16_t target[SERVO_COUNT]) {
    Serial.printf("[PoseCal] CMD65 target=%d,%d,%d,%d,%d,%d; status11/stream=%d,%d,%d,%d,%d,%d\n",
        target[0], target[1], target[2], target[3], target[4], target[5],
        g_last_servo_positions[0], g_last_servo_positions[1], g_last_servo_positions[2],
        g_last_servo_positions[3], g_last_servo_positions[4], g_last_servo_positions[5]);
    if (!verify_pose_near(target, POSE_LOCK_VERIFY_DELTA)) {
        Serial.println("[PoseCal] Pre-write position check failed; lock refused");
        arm.set_torque(false);
        return false;
    }
    uint8_t ids[6] = {1, 2, 3, 4, 5, 6};
    int16_t positions[6], speeds[6] = {80, 80, 80, 80, 80, 80};
    uint8_t accs[6] = {20, 20, 20, 20, 20, 20};
    memcpy(positions, target, sizeof(positions));
    servo.sync_write_pos_speed_ex(ids, positions, speeds, accs, SERVO_COUNT);
    servo.uart->flush();
    delay(100);
    if (!verify_pose_near(target, POSE_LOCK_VERIFY_DELTA)) {
        Serial.println("[PoseCal] Target preparation position check failed; lock refused");
        arm.set_torque(false);
        return false;
    }
    arm.board.oled.set_custom_text(1, "Hold: checking lock");
    arm.board.oled.show_custom();
    arm.set_torque(true);
    uint32_t started = millis();
    uint16_t samples = 0;
    do {
        int16_t actual[6];
        if (!request_fresh_servo_positions(actual, 350)) {
            arm.set_torque(false);
            Serial.println("[PoseCal] Feedback lost while locking; torque OFF");
            return false;
        }
        for (uint8_t i = 0; i < SERVO_COUNT; ++i) {
            int32_t delta = static_cast<int32_t>(actual[i]) - target[i];
            if (abs(delta) > POSE_LOCK_VERIFY_DELTA) {
                arm.set_torque(false);
                Serial.printf("[PoseCal] LOCK DRIFT id=%u target=%d actual=%d delta=%ld; torque OFF\n",
                    i + 1, target[i], actual[i], static_cast<long>(delta));
                return false;
            }
        }
        ++samples;
        delay(50);
    } while (millis() - started < 2000);
    if (samples < 5) {
        arm.set_torque(false);
        Serial.println("[PoseCal] Insufficient lock samples; torque OFF");
        return false;
    }
    Serial.printf("[PoseCal] CMD65 hold verified for 2s, samples=%u\n", samples);
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

    if (!lock_at_measured_pose(captured)) {
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

static void load_handle_teach() {
    Preferences storage;
    HandleTeachRecord loaded = {};
    bool ok = false;
    if (storage.begin("handle_real", true)) {
        ok = storage.getBytesLength("poses_v1") == sizeof(loaded) &&
             storage.getBytes("poses_v1", &loaded, sizeof(loaded)) == sizeof(loaded) &&
             valid_handle_record(loaded);
        storage.end();
    }
    if (ok) s_handle_record = loaded;
    Serial.printf("[HandleTeach] saved mask=0x%02X; playback DISABLED\n", s_handle_record.valid_mask);
}

static bool persist_handle_teach(const HandleTeachRecord& candidate) {
    if (!valid_handle_record(candidate)) return false;
    Preferences storage;
    if (!storage.begin("handle_real", false)) return false;
    bool ok = storage.putBytes("poses_v1", &candidate, sizeof(candidate)) == sizeof(candidate);
    storage.end();
    if (!ok || !storage.begin("handle_real", true)) return false;
    HandleTeachRecord readback = {};
    ok = storage.getBytesLength("poses_v1") == sizeof(readback) &&
         storage.getBytes("poses_v1", &readback, sizeof(readback)) == sizeof(readback) &&
         valid_handle_record(readback) && memcmp(&candidate, &readback, sizeof(candidate)) == 0;
    storage.end();
    return ok;
}

static void save_handle_teach_pose() {
    s_handle_error = nullptr;
    arm.board.oled.set_custom_text(1, "Hold still: reading");
    arm.board.oled.show_custom();
    int16_t captured[SERVO_COUNT];
    if (!capture_stable_servo_pose(captured)) {
        s_handle_error = "Unstable: retry K1";
        Serial.println("[HandleTeach] No stable fresh feedback; nothing saved");
        arm.board.buzzer.set(180, 80, 2, 900);
        show_handle_teach();
        return;
    }
    if (!lock_at_measured_pose(captured)) {
        arm.set_torque(false);
        s_handle_state = HT_RELEASED;
        s_handle_error = "Lock failed: OFF";
        Serial.println("[HandleTeach] Lock verification failed; torque OFF, support arm");
        arm.board.buzzer.set(200, 100, 3, 700);
        show_handle_teach();
        return;
    }
    HandleTeachRecord candidate = s_handle_record;
    // Re-teaching a point invalidates all later points of the previous path.
    candidate.valid_mask = static_cast<uint8_t>((1u << (s_handle_slot + 1)) - 1u);
    memcpy(candidate.positions[s_handle_slot], captured, sizeof(captured));
    for (uint8_t slot = s_handle_slot + 1; slot < 8; ++slot)
        memset(candidate.positions[slot], 0, sizeof(candidate.positions[slot]));
    candidate.checksum = handle_record_checksum(candidate);
    if (!persist_handle_teach(candidate)) {
        s_handle_state = HT_SAVE_FAILED; // Still held; never claim persistence succeeded.
        Serial.println("[HandleTeach] NVS write/readback failed; held, KEY1 retries");
        arm.board.buzzer.set(180, 80, 2, 900);
        show_handle_teach();
        return;
    }
    s_handle_record = candidate;
    s_handle_state = HT_HELD;
    Serial.printf("[HandleTeach] Saved slot=%u mask=0x%02X joints=%d,%d,%d,%d,%d,%d\n",
        s_handle_slot, candidate.valid_mask, captured[0], captured[1], captured[2],
        captured[3], captured[4], captured[5]);
    arm.board.buzzer.set(80, 60, 2, 2600);
    show_handle_teach();
}

static void handle_teach_keys(bool key1, bool key2) {
    if (!key1 && !key2) return; // OLED refresh remains on the periodic timer.
    if (key1 && key2) return; // Ambiguous simultaneous clicks do nothing.
    if (s_handle_state == HT_IDLE) {
        if (key1) {
            s_handle_slot = 0;
            s_handle_state = HT_CONFIRM; // Entering the menu does NOT release torque.
        } else if (key2) {
            Serial.println("[HandleTeach] RUN blocked: this build only records poses");
            arm.board.buzzer.set(100, 100, 2, 900);
        }
    } else if (s_handle_state == HT_CONFIRM) {
        if (key1) {
            arm.set_torque(false);
            s_handle_state = HT_RELEASED;
            s_handle_error = nullptr;
            Serial.printf("[HandleTeach] slot=%u torque OFF; support and adjust\n", s_handle_slot);
        } else if (key2) s_handle_state = HT_IDLE; // Holding state unchanged.
    } else if (s_handle_state == HT_RELEASED) {
        if (key1) save_handle_teach_pose(); // KEY2 cannot skip an unsaved point.
    } else if (s_handle_state == HT_SAVE_FAILED) {
        if (key1) save_handle_teach_pose();
        else if (key2) s_handle_state = HT_CONFIRM;
    } else if (s_handle_state == HT_HELD) {
        if (key1) s_handle_state = HT_CONFIRM;
        else if (key2) {
            if (s_handle_slot == 7) s_handle_state = HT_DONE;
            else { ++s_handle_slot; s_handle_state = HT_CONFIRM; }
        }
    } else if (s_handle_state == HT_DONE && key2) s_handle_state = HT_IDLE;
    apply_oled_standby_screen();
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

// All hardware/UI servicing runs on the Arduino loop task. U8g2 and its
// String buffers must not be accessed concurrently from an event-loop task.
static void service_periodic_tasks(void)
{
    const uint32_t now = millis();
    static uint32_t last_buzzer = 0, last_bat = 0, last_status = 0, last_oled = 0;
    if (now - last_buzzer >= BUZZER_UPDATE_PERIOD) {
        last_buzzer = now;
        arm.board.buzzer.update();
    }
    if (now - last_bat >= BAT_UPDATE_PERIOD) {
        last_bat = now;
        arm.board.bat.update();
    }
    if (now - last_status >= SERVO_STATUS_UPDATE_PERIOD) {
        last_status = now;
        arm.update_status();
    }
    if (now - last_oled >= OLED_REFRESH_PERIOD) {
        last_oled = now;
        apply_oled_standby_screen();
    }
}

static void func_ctrl_callback(PacketTypeDef* self)
{
    uint8_t len;
    uint16_t bat_level;

    // Teaching build accepts read-only host requests. Legacy RUN/JOG/download
    // commands must not interrupt manual support or bypass the teach UI.
    if (self->elements.id != 0xFF) return;
    switch (self->elements.cmd) {
        case CMD_GET_REAL_JOINT_ANGLES:
            servo.tx_frame_write(0xFF, CMD_GET_REAL_JOINT_ANGLES, nullptr, 0);
            return;
        case CMD_FIRMWARE_VERSION_CHECK:
        case CMD_CHECK_BAT_LEVEL_CHECK:
        case CMD_COLOR_GET_PLACE:
        case CMD_COLOR_GET_PICK:
        case CMD_COLOR_GET_POSE:
            break;
        case 105: { // Read the versioned, checksummed teaching record; no motion.
            len = serial_port.protocol.tx_packet_complete(0xFF, 105,
                reinterpret_cast<uint8_t*>(&s_handle_record), sizeof(s_handle_record));
            serial_port.uart->write(reinterpret_cast<const uint8_t*>(&serial_port.protocol.tx_packet), len);
            return;
        }
        default:
            Serial.printf("[HandleTeach] Host cmd %u blocked in teach-only build\n", self->elements.cmd);
            return;
    }

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
                    else if (color_id == 4) cName = "yellow";
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

        /* ── 比赛适配 ①：一次下发整个颜色序列 + 每个颜色的放置槽位 ──
         * payload: [count] + count x { [nameLen][name...][placeSlot] }
         * count == 0 表示停止。
         * 例：任务二 口令 蓝-红-黄 → count=3, {3,"blue",1}{3,"red",2}{6,"yellow",3}
         * 例：任务一 抓红色放 T0  → count=1, {3,"red",0} */
        case CMD_COLOR_GRAB_SEQ: {
            uint16_t alen = (uint16_t)(self->elements.length - 2);
            if (alen < 1) break;

            uint8_t n = self->elements.args[0];
            if (n == 0) {
                colorGrabber.stop();
                arm.board.buzzer.off();
                Serial.println("[ColorSort] SEQ stop");
                break;
            }
            if (n > 8) n = 8;

            static char        namebuf[8][16];
            static uint8_t     slots[8];
            static const char* names[8];

            uint16_t off = 1;
            uint8_t  got = 0;
            bool     bad = false;

            for (uint8_t i = 0; i < n; i++) {
                if (off + 1 > alen) { bad = true; break; }
                uint8_t nl = self->elements.args[off++];
                if (nl == 0 || nl > 15 || (uint32_t)off + nl + 1 > alen) { bad = true; break; }
                memcpy(namebuf[i], &self->elements.args[off], nl);
                namebuf[i][nl] = '\0';
                off += nl;
                slots[i] = (uint8_t)(self->elements.args[off++] & 0x07);
                names[i] = namebuf[i];
                got++;
            }

            if (bad || got != n) {
                Serial.printf("[ColorSort] SEQ payload invalid (n=%d got=%d)\n", n, got);
                arm.board.buzzer.set(60, 200, 1, 1500);
                break;
            }

            colorGrabber.stop();
            delay(100);
            colorGrabber.startSeq(names, slots, n);
            arm.board.buzzer.set(100, 100, 2, 2000);
        } break;

        /* ── 比赛适配 ②：设置某个放置槽位的坐标（立即写入 Flash）──
         * payload: [slot:u8][x:f32][y:f32][z:f32] 全部小端 */
        case CMD_COLOR_SET_PLACE: {
            uint16_t alen = (uint16_t)(self->elements.length - 2);
            if (alen >= 13) {
                uint8_t slot = (uint8_t)(self->elements.args[0] & 0x07);
                float x = 0, y = 0, z = 0;
                memcpy(&x, &self->elements.args[1], 4);
                memcpy(&y, &self->elements.args[5], 4);
                memcpy(&z, &self->elements.args[9], 4);
                colorGrabber.setPlaceSlot(slot, x, y, z);
                save_place_slots();
                arm.board.buzzer.set(60, 40, 1, 3000);
                Serial.printf("[ColorSort] SET_PLACE slot %d = (%.1f, %.1f, %.1f)\n", slot, x, y, z);
            }
        } break;

        /* ── 比赛适配 ③：读回全部 8 个放置槽位 ──
         * 返回 8 槽 x (x,y,z) 的 float32 小端 = 96 字节 */
        case CMD_COLOR_GET_PLACE: {
            static uint8_t pbuf[8 * 3 * 4];
            memcpy(pbuf, (const uint8_t*)colorGrabber.placeTableRaw(), sizeof(pbuf));
            len = serial_port.protocol.tx_packet_complete(0xFF, CMD_COLOR_GET_PLACE, pbuf, sizeof(pbuf));
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
        } break;

        /* ── 比赛适配 ④：示教某颜色的“抓取源坐标” ──
         * payload: [nameLen:u8][name...][x:f32][y:f32][z:f32]
         * 坐标含义 = 夹爪夹住电池时末端应该在的位置（抓取点），不是相机居中的位置。
         * 颜色名固定支持 red / green / blue / yellow。 */
        case CMD_COLOR_SET_PICK: {
            uint16_t alen = (uint16_t)(self->elements.length - 2);
            if (alen >= 14) {
                uint8_t nl = self->elements.args[0];
                if (nl >= 1 && nl <= 15 && (uint32_t)1 + nl + 12 <= alen) {
                    char nm[16];
                    memcpy(nm, &self->elements.args[1], nl);
                    nm[nl] = '\0';
                    float x = 0, y = 0, z = 0;
                    memcpy(&x, &self->elements.args[1 + nl], 4);
                    memcpy(&y, &self->elements.args[5 + nl], 4);
                    memcpy(&z, &self->elements.args[9 + nl], 4);
                    if (colorGrabber.setPickSlot(nm, x, y, z)) {
                        save_pick_table();
                        arm.board.buzzer.set(60, 40, 1, 3000);
                        Serial.printf("[ColorSort] SET_PICK %s = (%.1f, %.1f, %.1f)\n", nm, x, y, z);
                    } else {
                        Serial.printf("[ColorSort] SET_PICK 未知颜色 '%s'\n", nm);
                        arm.board.buzzer.set(60, 200, 1, 1500);
                    }
                }
            }
        } break;

        /* ── 比赛适配 ⑤：读回 4 个颜色的抓取源坐标（固定顺序 red,green,blue,yellow）──
         * 返回 4 x (x,y,z) float32 小端 = 48 字节 */
        case CMD_COLOR_GET_PICK: {
            static uint8_t kbuf[4 * 3 * 4];
            memcpy(kbuf, (const uint8_t*)colorGrabber.pickTableRaw(), sizeof(kbuf));
            len = serial_port.protocol.tx_packet_complete(0xFF, CMD_COLOR_GET_PICK, kbuf, sizeof(kbuf));
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
        } break;

        /* ── 比赛适配 ⑥：设置抓取模式 ── payload: [mode:u8]
         * 0 = 视觉优先，超时降级到坐标（默认）
         * 1 = 强制走坐标，不用视觉
         * 2 = 强制只用视觉，不降级 */
        case CMD_COLOR_SET_PICKMODE: {
            if (self->elements.length >= 3) {
                colorGrabber.setPickMode(self->elements.args[0]);
                save_pick_table();
                arm.board.buzzer.set(60, 40, 1, 3000);
                Serial.printf("[ColorSort] pickmode = %d\n", colorGrabber.getPickMode());
            }
        } break;

        /* ── 比赛适配 ⑦：读回当前末端位姿（标定时看实时 XYZ）──
         * 无参；返回 6 x f32 = 24 字节：x, y, z, pitch, roll, claw
         * 说明：原固件根本没有向主机暴露位姿，所以这里自己加。 */
        case CMD_COLOR_GET_POSE: {
            arm.update_status();
            uint32_t t0 = millis();
            while (millis() - t0 < 150) {          // 等 AT32 回包刷新 current_pose
                pump_at32_feedback();
                delay(5);
            }
            float pv[6];
            pv[0] = arm.current_pose.x;
            pv[1] = arm.current_pose.y;
            pv[2] = arm.current_pose.z;
            pv[3] = arm.current_pose.pitch;
            pv[4] = arm.current_pose.roll;
            pv[5] = arm.current_pose.claw;
            uint8_t pb[sizeof(pv)];
            memcpy(pb, pv, sizeof(pv));
            len = serial_port.protocol.tx_packet_complete(0xFF, CMD_COLOR_GET_POSE, pb, sizeof(pb));
            serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);
        } break;

        /* ── 比赛适配 ⑧：相对点动（标定微调用）──
         * payload: [dx][dy][dz][dpitch][droll][dclaw] 共 6 个 f32 = 24 字节
         * 实现方式：先刷新当前位姿，再做“当前位置 + 增量”的绝对运动。
         * 走的是 CMD_COORDINATE_SET（已被 ColorGrabber 大量验证可用），
         * 比直接用 AT32 的相对运动指令 CMD_ARM_MOVE_INC 更稳妥。
         * 会夹到 Robot_Arm 的坐标限位内。回返回新的目标位姿 6 x f32。 */
        case CMD_COLOR_JOG: {
            uint16_t alen = (uint16_t)(self->elements.length - 2);
            if (alen >= 24) {
                float d[6];
                memcpy(d, self->elements.args, sizeof(d));

                arm.update_status();
                uint32_t t0 = millis();
                while (millis() - t0 < 120) {
                    pump_at32_feedback();
                    delay(5);
                }

                float nx = constrain(arm.current_pose.x + d[0], arm.limit_x_min, arm.limit_x_max);
                float ny = constrain(arm.current_pose.y + d[1], arm.limit_y_min, arm.limit_y_max);
                float nz = constrain(arm.current_pose.z + d[2], arm.limit_z_min, arm.limit_z_max);
                float np = arm.current_pose.pitch + d[3];
                float nr = arm.current_pose.roll + d[4];
                float nc = constrain(arm.current_pose.claw + d[5], -80.0f, 60.0f);

                arm.move(nx, ny, nz, np, nr, nc, 400);

                float pv[6] = { nx, ny, nz, np, nr, nc };
                uint8_t pb[sizeof(pv)];
                memcpy(pb, pv, sizeof(pv));
                len = serial_port.protocol.tx_packet_complete(0xFF, CMD_COLOR_JOG, pb, sizeof(pb));
                serial_port.uart->write((const uint8_t*)&serial_port.protocol.tx_packet, len);

                Serial.printf("[ColorSort] JOG -> (%.1f, %.1f, %.1f) p=%.1f r=%.1f c=%.1f\n",
                              nx, ny, nz, np, nr, nc);
            }
        } break;

        default:
            break;
    }
}

void at32_packet_callback(PacketTypeDef* rx_packet)
{
    if (is_downloading) return;

    if (rx_packet->elements.id == 0x5A &&
        rx_packet->elements.cmd == CMD_GET_REAL_JOINT_ANGLES) {
        if (rx_packet->elements.length == 26 &&
            decode_real_joint_positions(rx_packet->elements.args, 24, g_real_joint_positions)) {
            ++g_real_joint_sequence;
        } else {
            Serial.println("[PoseCal] Rejected malformed/out-of-range CMD65 feedback");
        }
    }

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
        service_periodic_tasks();

        if (s_pose_calibration_active) {
            if (arm.board.button.is_clicked(KEY1_BUTTON_ID)) {
                Serial.println("[Keys] KEY1 GPIO0 short click -> save current pose");
                save_and_lock_manual_pose();
            }
            return;
        }

        if (s_handle_state == HT_IDLE && arm.board.button.is_long_clicked(KEY1_BUTTON_ID)) {
            enter_pose_calibration("KEY1 long press");
            return;
        }
        handle_teach_keys(arm.board.button.is_clicked(KEY1_BUTTON_ID),
                          arm.board.button.is_clicked(KEY2_BUTTON_ID));
        // Automatic grabber is deliberately not serviced by the teaching build.
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

    if (!protocol_self_test()) {
        Serial.println("[ProtocolTest] FAIL; hardware initialization stopped");
        while (true) delay(1000);
    }
    Serial.println("[ProtocolTest] PASS: 1000 interleaved pairs, checksum, reentrancy, TX bounds");

    if (!handle_record_self_test()) {
        Serial.println("[HandleTeachTest] FAIL; hardware initialization stopped");
        while (true) delay(1000);
    }
    Serial.println("[HandleTeachTest] PASS: record CRC, range, stage mask, version");

    if (!real_joint_feedback_self_test()) {
        Serial.println("[RealJointTest] FAIL; hardware initialization stopped");
        while (true) delay(1000);
    }
    Serial.println("[RealJointTest] PASS: CMD65 pulse stride, length, range, atomic decode");

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
    load_place_slots();
    load_pick_table();
    load_handle_teach();

    // Start released; goal registers are prepared before torque can be enabled.
    arm.set_torque(false);
    delay(100);

    // Diagnostic build: never restore a stale saved pose automatically.
    // The operator must support the arm and confirm the current pose using KEY1.
    enter_pose_calibration("startup requires manual confirmation");

    Serial.printf("[%s] Color sort firmware %s ready\n", TAG, FIRMWARE_VERSION_STR);
}
