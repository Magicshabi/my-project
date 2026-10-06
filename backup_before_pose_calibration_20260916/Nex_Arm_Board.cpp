#include "Nex_Arm_Board.h"
#include "U8g2lib.h"
#include "Wire.h"
#include "HX_30HM.h"   
#include "Global.h"
#include "usb_ctrl.h"  
#include "Robot_Arm.h"
#include "system_task_handle.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BOOT_BUTTON_PIN         0
#define USER_BUTTON_PIN         2
#define BUZZER_PIN              23
#define I2C_SDA_PIN             26
#define I2C_SCL_PIN             27
#define MOTOR_SDA_PIN           21
#define MOTOR_SCL_PIN           22
#define ADC_PIN                 ADC1_CHANNEL_6
#define ADC_WIDTH               ADC_WIDTH_12Bit
#define ADC_ATTEN               ADC_ATTEN_DB_2_5
#define DEFAULT_VREF            1100
#define OLED_I2C_ADDR           0x3C

PinButton Boot_Button(BOOT_BUTTON_PIN);
PinButton User_Button(USER_BUTTON_PIN);
U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R2, U8X8_PIN_NONE, I2C_SCL_PIN, I2C_SDA_PIN);

bool OLED_t::begin()
{
    Wire.setPins(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.begin();
    Wire.setClock(400000);
    Wire.beginTransmission(OLED_I2C_ADDR);
    
    if(!Wire.endTransmission()) {
        u8g2.begin();
        return true;
    }
    return false;
}

void OLED_t::set_custom_text(uint8_t line, String text)
{
    if(line < 4) {
        custom_lines[line] = text; 
    }
}

void OLED_t::set_icon(uint8_t icon_id) {
    current_icon = icon_id;
}

void OLED_t::show_icon() {
    u8g2.firstPage();
    do {
        uint32_t t = millis();
        bool is_blinking = (t % 3000) < 150;

        if (current_icon == 1) {
            if (is_blinking) {
                u8g2.drawBox(40, 24, 8, 2);
                u8g2.drawBox(80, 24, 8, 2);
            } else {
                u8g2.drawDisc(44, 24, 4);
                u8g2.drawDisc(84, 24, 4);
            }
            u8g2.drawCircle(64, 32, 16, U8G2_DRAW_LOWER_RIGHT | U8G2_DRAW_LOWER_LEFT);
        }
        else if (current_icon == 2) {
            if (is_blinking) {
                u8g2.drawBox(40, 28, 8, 2);
                u8g2.drawBox(80, 28, 8, 2);
            } else {
                u8g2.drawDisc(44, 28, 4);
                u8g2.drawDisc(84, 28, 4);
            }
            u8g2.drawCircle(64, 48, 12, U8G2_DRAW_UPPER_RIGHT | U8G2_DRAW_UPPER_LEFT);

            int tear_y = 36 + ((t / 50) % 10);
            u8g2.drawBox(42, tear_y, 4, 6);
            u8g2.drawBox(82, tear_y, 4, 6);
        }
        else if (current_icon == 3) {
            if (is_blinking) {
                u8g2.drawBox(40, 24, 8, 2);
                u8g2.drawBox(80, 24, 8, 2);
            } else {
                u8g2.drawDisc(44, 24, 5);
                u8g2.drawDisc(84, 24, 5);
            }

            int mouth_r = 6 + ((t / 150) % 4);
            u8g2.drawDisc(64, 44, mouth_r);
        }
        else if (current_icon == 4) {
            // 顺时针旋转 loading 动画
            int cx = 64, cy = 32, r = 18;
            int seg = (t / 120) % 12;  // 12 段，每段 30 度，120ms 转一格

            // 画 12 段圆弧刻度，当前段加粗高亮
            for (int i = 0; i < 12; i++) {
                float angle = (float)i * 30.0f * 3.14159f / 180.0f;
                int x1 = cx + (int)((r - 6) * sinf(angle));
                int y1 = cy - (int)((r - 6) * cosf(angle));
                int x2 = cx + (int)(r * sinf(angle));
                int y2 = cy - (int)(r * cosf(angle));

                // 距离当前段越近越亮（画越粗的线）
                int dist = (seg - i + 12) % 12;
                if (dist < 4) {
                    u8g2.drawLine(x1, y1, x2, y2);
                    if (dist < 2) {
                        u8g2.drawLine(x1 + 1, y1, x2 + 1, y2);
                        u8g2.drawLine(x1 - 1, y1, x2 - 1, y2);
                    }
                }
            }
        }
    } while (u8g2.nextPage());
}

void OLED_t::show_custom()
{
    u8g2.firstPage();
    do {
        // 第一行用粗体大字（和 WiFi AP 页面一致）
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(0, 14);
        u8g2.print(custom_lines[0]);

        // 后续行用普通大字
        u8g2.setFont(u8g2_font_7x14_tr);
        u8g2.setCursor(0, 34);
        u8g2.print(custom_lines[1]);
        u8g2.setCursor(0, 50);
        u8g2.print(custom_lines[2]);
        u8g2.setCursor(0, 63);
        u8g2.print(custom_lines[3]);
        
    } while (u8g2.nextPage());
}

void OLED_t::show_status(float x, float y, float z, float pitch, float roll, float claw)
{
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(0, 12);
        u8g2.print("Arm Status");
        
        u8g2.setFont(u8g2_font_6x10_tr);
        u8g2.setCursor(0, 26);
        u8g2.print("X:"); u8g2.print(x, 0); u8g2.print("mm");
        u8g2.setCursor(64, 26);
        u8g2.print("Y:"); u8g2.print(y, 0); u8g2.print("mm");
        
        u8g2.setCursor(0, 40);
        u8g2.print("Z:"); u8g2.print(z, 0); u8g2.print("mm");
        u8g2.setCursor(64, 40);
        u8g2.print("P:"); u8g2.print(pitch, 1); u8g2.print("d");
        
        u8g2.setCursor(0, 54);
        u8g2.print("R:"); u8g2.print(roll, 0); u8g2.print("d");
        u8g2.setCursor(64, 54);
        u8g2.print("C:"); u8g2.print(claw, 0); u8g2.print("d");
        
    } while (u8g2.nextPage());
}

void OLED_t::show_espnow_info(uint8_t channel, uint8_t acc, bool is_unicast, String mac)
{
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(0, 14);
        u8g2.print("[ESP-NOW]");

        u8g2.setFont(u8g2_font_7x14_tr);
        u8g2.setCursor(0, 34);
        u8g2.print("CH:"); u8g2.print(channel);
        u8g2.print("  ACC:"); u8g2.print(acc);

        u8g2.setCursor(0, 54);
        u8g2.print(mac);
        
    } while (u8g2.nextPage());
}

void OLED_t::show_wifi_ap_info(String ssid, IPAddress ip)
{
    u8g2.firstPage();
    do {
        u8g2.setFont(u8g2_font_7x14B_tr);
        u8g2.setCursor(0, 14);
        u8g2.print("[WiFi AP]");

        u8g2.setFont(u8g2_font_7x14_tr);
        u8g2.setCursor(0, 36);
        u8g2.print(ssid);

        u8g2.setCursor(0, 58);
        u8g2.print(ip);
        
    } while (u8g2.nextPage());
}

void Bat_t::begin()
{
    adc1_config_channel_atten(ADC_PIN, ADC_ATTEN);
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, DEFAULT_VREF, &adc_chars);
    update();
}

void Bat_t::update()
{
    int raw = adc1_get_raw(ADC_PIN);
    int samples_voltage = esp_adc_cal_raw_to_voltage(raw, &adc_chars);
    samples_voltage = (int)(11.0f * (float)samples_voltage);
    filter_buf[filter_index] = samples_voltage;
    filter_index = (filter_index + 1) % WINDOWS_SIZE;

    int sum = 0;
    if(filter_buf[WINDOWS_SIZE - 1] != 0) {
        for (uint8_t i = 0; i < WINDOWS_SIZE; i++) sum += filter_buf[i];
        voltage = sum / WINDOWS_SIZE;
    }
}

int Bat_t::get_voltage()
{
    return voltage;
}

void Button_t::update()
{
    Boot_Button.update();
    User_Button.update();
}

bool Button_t::is_clicked(uint8_t id)
{
    return id == 0 ? Boot_Button.isClick() : User_Button.isClick();
}

void Buzzer_t::update()
{
    switch(stage) {
    case BUZZER_STAGE_START_NEW_CYCLE:
        if(ticks_on > 0) {
            tone(BUZZER_PIN, freq);
            ticks_count = 0;
            stage = BUZZER_STAGE_WATTING_OFF;
        } else {
            noTone(BUZZER_PIN);
            stage = BUZZER_STAGE_IDLE;
        }
        break;
    case BUZZER_STAGE_WATTING_OFF:
        ticks_count += update_period;
        if(ticks_count >= ticks_on) {
            noTone(BUZZER_PIN);
            if(ticks_off > 0) {
                stage = BUZZER_STAGE_WATTING_PERIOD_END;
            } else {
                // off_time=0 时直接结束，不再循环
                stage = BUZZER_STAGE_IDLE;
            }
        }
        break;
    case BUZZER_STAGE_WATTING_PERIOD_END:
        ticks_count += update_period;
        if(ticks_count >= (ticks_off + ticks_on)) {
            ticks_count -= (ticks_off + ticks_on);
            if(times == 1) {
                noTone(BUZZER_PIN);
                stage = BUZZER_STAGE_IDLE;
            } else {
                tone(BUZZER_PIN, freq);
                times = times == 0 ? 0 : times - 1;
                stage = BUZZER_STAGE_WATTING_OFF;
            }
        }
        break;
    case BUZZER_STAGE_IDLE:
        break;
    }
}    

void Buzzer_t::begin()
{
    allow_change = true;
    
    this->ticks_on = 0;
    this->ticks_off = 0;
    this->times = 0;
    this->freq = 2000;

    stage = BUZZER_STAGE_IDLE;  // 上电默认静音，不响
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
    ledcAttachPin(BUZZER_PIN, 0);
    noTone(BUZZER_PIN);
}

bool Buzzer_t::on(uint16_t freq)
{
  if(allow_change) {
    this->ticks_on = 1;
    this->ticks_off = 0;
    this->times = 0;
    this->freq = freq;
    this->stage = BUZZER_STAGE_START_NEW_CYCLE;
    return true;
  }
  return false;
}

bool Buzzer_t::off(void)
{
  if(allow_change) {
    this->ticks_on = 0;
    this->ticks_off = 1;
    this->times = 0;
    this->stage = BUZZER_STAGE_START_NEW_CYCLE;
    return true;
  }
  return false;
}

bool Buzzer_t::set(uint32_t on_time , uint32_t off_time , uint16_t times, uint16_t freq)
{
  if(allow_change) {
    this->ticks_on = on_time;
    this->ticks_off = off_time;
    this->times = times;
    this->freq = freq;
    this->stage = BUZZER_STAGE_START_NEW_CYCLE;
    return true;
  }
  return false;
}

void HW_Board::begin()
{
  if(!LittleFS.begin(true)) {
  } 
  bat.begin();
  buzzer.begin();
  oled.begin();
  motor.begin(Wire1, MOTOR_SDA_PIN, MOTOR_SCL_PIN, MOTOR_JGB37_520, 0);
  mecanum.begin(&motor);
  tank.begin(&motor);
  conveyor.begin(&Wire1);
  stepper.begin(&Wire1);
}

void HW_Board::list_action_group_dir()
{
  File root = LittleFS.open("/");
  File file = root.openNextFile();
  while(file) {
    file = root.openNextFile();
  }
}

void HW_Board::action_group_run(uint8_t id)
{
  uint8_t total_frames = 0;
  const uint8_t FRAME_LEN = 40; 
  uint8_t buf[40] = {0}; 

  String fileName = "/ActionGroup" + String(id) + ".rob";

  if(!LittleFS.exists(fileName)) {
      return;
  }

  File file = LittleFS.open(fileName, "r");
  if(!file) return;

  if(file.available()) {
      total_frames = file.read();
  } else {
      file.close(); return;
  }
  
  act_state = READ_FRAME_NUM; 
  int16_t current_pos[6];
  bool has_feedback = sync_arm_feedback(80) && get_last_servo_positions(current_pos);

  if(!has_feedback) {
      for(uint8_t i = 0; i < 6; ++i) {
          current_pos[i] = 2048;
      }
  }

  while(file.available()) {
      if(act_state == ACT_STOP) {
          break;
      }

      if (file.read(buf, FRAME_LEN) != FRAME_LEN) {
          break;
      }

      uint8_t frame_index = buf[0];
      uint8_t servo_num = buf[1];
      uint16_t move_time = BYTE_TO_HW(buf[3], buf[2]); 

      if(servo_num > 6) {
          servo_num = 6;
      }

      uint8_t ids[6];
      int16_t positions[6];
      int16_t speeds[6];
      uint8_t accs[6];
      uint8_t valid_num = 0;

      for(uint8_t i = 0; i < servo_num; i++) {
        uint8_t base_idx = 4 + i * 6;
        uint8_t s_id = buf[base_idx];
        uint16_t s_pos = BYTE_TO_HW(buf[base_idx + 2], buf[base_idx + 1]);
        uint8_t s_acc = buf[base_idx + 3];

        if(s_id >= 1 && s_id <= 6) {
            int32_t distance = abs((int32_t)s_pos - (int32_t)current_pos[s_id - 1]);
            int32_t calc_speed = 10;

            if(move_time > 0) {
                calc_speed = (int32_t)(distance * 1000.0f / (float)move_time * 1.2f);
            } else {
                calc_speed = 3400;
            }

            if(calc_speed < 10) calc_speed = 10;
            if(calc_speed > 3400) calc_speed = 3400;

            ids[valid_num] = s_id;
            positions[valid_num] = (int16_t)s_pos;
            speeds[valid_num] = (int16_t)calc_speed;
            accs[valid_num] = s_acc;
            current_pos[s_id - 1] = (int16_t)s_pos;
            valid_num++;
        }
      }

      if(valid_num > 0) {
          servo.sync_write_pos_speed_ex(ids, positions, speeds, accs, valid_num);
      }

      arm.update_status();
      TickType_t wait_start = xTaskGetTickCount();
      TickType_t wait_ticks = pdMS_TO_TICKS(move_time);
      while((xTaskGetTickCount() - wait_start) < wait_ticks) {
          vTaskDelay(pdMS_TO_TICKS(5));
          extern SerialPort_t serial_port; 
          serial_port.rec_handler();
          pump_at32_feedback();
          if(act_state == ACT_STOP) break;
      }

      if(frame_index == total_frames) break;
  }
  
  file.close();
  act_state = READ_FRAME_NUM; 
}

bool HW_Board::action_group_download(uint8_t id, uint8_t *data, size_t length)
{
  size_t written;
  size_t len;
  uint8_t frame_index = data[2];

  if(id > 255) return false; 
  String fileName = "/ActionGroup" + String(id) + ".rob";
  
  if(frame_index == 1) {
    File file = LittleFS.open(fileName, "w");
    if(!file) {
        return false;
    }
    file.write(data[1]); 
    len = length - 2;
    written = file.write(&data[2], len);
    file.close();
  } else {
    File file = LittleFS.open(fileName, "a");
    if(!file) return false;
    len = length - 2;
    written = file.write(&data[2], len);
    file.close();
  }
  return true;
}

bool HW_Board::action_group_erase(uint8_t id)
{
    String fileName = "/ActionGroup" + String(id) + ".rob";
    if(LittleFS.exists(fileName)) {
        LittleFS.remove(fileName);
        return true;
    }
    return false;
}

void HW_Board::action_group_stop(void)
{
    act_state = ACT_STOP;
}