#ifndef NEX_ARM_BOARD_H_
#define NEX_ARM_BOARD_H_

#include "Arduino.h"
#include "PinButton.h"
#include "FS.h"
#include "LittleFS.h" 
#include "esp_adc_cal.h"
#include "driver/adc.h"
#include "Motor_I2C.h"
#include "Mecanum_Chassis.h"
#include "Tank_Chassis.h"
#include "Conveyor_Belt.h"
#include "Stepper_Strip.h"
#include "WiFi.h" 

#define WINDOWS_SIZE 10

class OLED_t {
public:
    String custom_lines[4] = {"", "", "", ""}; 
    uint8_t current_icon = 0;

    bool begin();
    // void show_status(float x, float y, float z);
    // void show_status(float x, float y, float z, float pitch, float roll);
    void show_status(float x, float y, float z, float pitch, float roll, float claw);

    void show_espnow_info(uint8_t channel, uint8_t acc, bool is_unicast, String mac);
    void show_wifi_ap_info(String ssid, IPAddress ip); 
    
    void set_custom_text(uint8_t line, String text);
    void show_custom();
    void set_icon(uint8_t icon_id);
    void show_icon();
};
class Bat_t {
public:
    void begin(void);
    void update(void);
    int get_voltage(void);

private:
    uint8_t filter_index = 0;
    int voltage;
    int filter_buf[WINDOWS_SIZE] = {0};
    esp_adc_cal_characteristics_t adc_chars;
};

class Button_t {
public:
  void update(void);
  bool is_clicked(uint8_t id);
};

class Buzzer_t {
public:
    bool allow_change = true;
    void begin(void);
    void update(void);
    bool off(void);
    bool on(uint16_t freq = 2000);
    bool set(uint32_t on_time, uint32_t off_time, uint16_t times, uint16_t freq = 2000);

private:
    uint16_t freq;
    uint16_t times;
    uint32_t ticks_on;
    uint32_t ticks_off;
    uint32_t ticks_count;
    const uint32_t update_period = 20;

    enum Stage {BUZZER_STAGE_START_NEW_CYCLE, 
                BUZZER_STAGE_WATTING_OFF, 
                BUZZER_STAGE_WATTING_PERIOD_END, 
                BUZZER_STAGE_IDLE};

    Stage stage = BUZZER_STAGE_IDLE;
};

class HW_Board {
public:
    Bat_t bat;
    OLED_t oled;
    Buzzer_t buzzer;
    Button_t button;
    Motor_I2C motor;
    Mecanum_Chassis mecanum;     
    Tank_Chassis tank;  
    Conveyor_Belt conveyor;
    Stepper_Strip stepper;
    enum Act_State {READ_FRAME_NUM, READ_FRAME_DATA, ACT_STOP};
    Act_State act_state = READ_FRAME_NUM;

    void begin(void);
    void list_action_group_dir(void);
    bool action_group_erase(uint8_t id);
    void action_group_run(uint8_t id);
    void action_group_stop(void);
    bool action_group_download(uint8_t id, uint8_t *data, size_t length);

private:  
    File file;
    uint8_t act_read_frame_num;
};

#endif