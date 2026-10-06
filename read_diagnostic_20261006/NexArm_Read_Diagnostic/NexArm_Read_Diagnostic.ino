#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include "CommProtocol.h"
#include "ProtocolSelfTest.h"
#include "RealJointFeedback.h"
#include "RecordJson.h"
#include "ReadPolicy.h"

// Only documented read requests; no key or host input can initiate motion.
SET_LOOP_TASK_STACK_SIZE(16384);
static CommProtocol_t rx,tx;
static HandleTeachRecord record;
static esp_adc_cal_characteristics_t adc_chars;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2,U8X8_PIN_NONE,27,26);
static bool ready=false,found=false,waiting=false,done=false;
static uint8_t slot=0,round_no=0;
static uint32_t sent=0,last_rx=0,last_query=0,last_report=0,next_query=1500;
static unsigned seen=0,adc_method=0;
static int supply_mv=0;

static bool transmit(uint8_t id,uint8_t cmd,const uint8_t* a,size_t n){
    if(!read_allowed(id,cmd,a,n)){ ready=false; Serial.println("[ReadDiag] TX BLOCKED"); return false; }
    uint16_t len=tx.tx_packet_complete(id,cmd,a,n);
    return len && Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet),len)==len;
}
static void receive(PacketTypeDef* p){
    size_t n=p->elements.length-2;
    int16_t pos[6];
    if(p->elements.id==0x5A && p->elements.cmd==65 && decode_real_joint_positions(p->elements.args,n,pos)){
        Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",(unsigned long)millis(),pos[0],pos[1],pos[2],pos[3],pos[4],pos[5]);
        return;
    }
    // Preserve every non-position frame, including error statuses and late replies.
    char hex[2*MAX_ARGS_SIZE+1];
    for(size_t i=0;i<n;++i) snprintf(hex+2*i,3,"%02x",p->elements.args[i]);
    hex[2*n]=0;
    bool candidate=waiting && ((queries[slot].id==255 && p->elements.id==0x5A && p->elements.cmd==queries[slot].cmd) ||
                              (queries[slot].id==6 && p->elements.id==6));
    if(candidate) ++seen;
    Serial.printf("{\"type\":\"diag_rx\",\"ms\":%lu,\"id\":%u,\"cmd_or_status\":%u,\"hex\":\"%s\",\"round\":%u,\"slot\":%u,\"within_query_window\":%s}\n",
        (unsigned long)millis(),p->elements.id,p->elements.cmd,hex,round_no,slot,candidate?"true":"false");
}
static void report(){
    int sum=0,low=4095,high=0;
    for(int i=0;i<32;++i){int x=adc1_get_raw(ADC1_CHANNEL_6);sum+=x;if(x<low)low=x;if(x>high)high=x;}
    int raw=sum/32; int pin_mv=esp_adc_cal_raw_to_voltage(raw,&adc_chars); supply_mv=pin_mv*11;
    Serial.printf("{\"type\":\"supply_estimate\",\"raw\":%d,\"raw_min\":%d,\"raw_max\":%d,\"pin_mv\":%d,\"estimated_board_mv\":%d,\"calibration_method\":%u,\"divider_from_factory\":11,\"servo_terminal_voltage_measured\":false}\n",raw,low,high,pin_mv,supply_mv,adc_method);
    char line[RECORD_JSON_SIZE];
    size_t n=format_record_json(record,"nexarm-read-diagnostic-r2","handle_free",false,line,sizeof(line));
    if(n) Serial.write(reinterpret_cast<const uint8_t*>(line),n);
    Serial.printf("{\"type\":\"diag_state\",\"done\":%s,\"round\":%u,\"slot\":%u,\"rx_bytes\":%lu,\"parse_errors\":%lu,\"motion_enabled\":false}\n",done?"true":"false",round_no,slot,(unsigned long)rx.stats.bytes,(unsigned long)rx.stats.errors);
    if(found){char v[24];snprintf(v,sizeof(v),"Board est: %d mV",supply_mv);oled.firstPage();do{
        oled.setFont(u8g2_font_6x12_tr);oled.drawStr(0,13,"READ DIAG r2");oled.drawStr(0,29,"NO MOTION COMMANDS");
        oled.drawStr(0,45,done?"Queries done":"Reading ID6");oled.drawStr(0,61,v);
    }while(oled.nextPage());}
}
void setup(){
    Serial.begin(1000000);delay(300);
    Wire.setPins(26,27);Wire.begin();Wire.setClock(400000);Wire.beginTransmission(0x3C);found=Wire.endTransmission()==0;if(found)oled.begin();
    if(!(read_policy_test() && protocol_boundary_self_test() && handle_record_self_test() && real_joint_feedback_self_test())){Serial.println("[ReadDiag] SELF TEST FAIL; UART disabled");return;}
    Serial.println("[ReadDiag] SELF TEST PASS");
    Preferences store;bool ok=store.begin("handle_free",true);
    if(ok){ok=store.getBytesLength("poses_v1")==sizeof(record) && store.getBytes("poses_v1",&record,sizeof(record))==sizeof(record) && valid_handle_record(record) && record.valid_mask==255;store.end();}
    if(!ok){Serial.println("[ReadDiag] INVALID RECORD; UART disabled");return;}
    adc1_config_width(ADC_WIDTH_BIT_12);adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_DB_2_5);
    adc_method=esp_adc_cal_characterize(ADC_UNIT_1,ADC_ATTEN_DB_2_5,ADC_WIDTH_BIT_12,1100,&adc_chars);
    rx.begin();tx.begin();rx.register_success_callback(receive);Serial1.setRxBufferSize(2048);Serial1.begin(1000000,SERIAL_8N1,17,16);
    ready=true;Serial.println("[ReadDiag] nexarm-read-diagnostic-r2 READY; no motion or torque writes");report();
}
void loop(){
    if(!ready){delay(10);return;}
    for(unsigned i=0;i<2048 && Serial1.available();++i){uint32_t now=millis();if(rx.has_partial_frame() && now-last_rx>100)rx.discard_partial_frame();last_rx=now;uint8_t b=Serial1.read();rx.parsing(&b,1);}
    for(unsigned i=0;i<256 && Serial.available();++i)Serial.read();
    uint32_t now=millis();if(rx.has_partial_frame() && now-last_rx>100)rx.discard_partial_frame();
    if(waiting && now-sent>=700){
        Serial.printf("{\"type\":\"diag_window_end\",\"round\":%u,\"slot\":%u,\"candidate_frames\":%u}\n",round_no,slot,seen);
        waiting=false;next_query=now+250;
        if(++slot==QUERY_COUNT){slot=0;if(++round_no==2)done=true;}
    }
    if(!done && !waiting && (int32_t)(now-next_query)>=0){
        const auto& q=queries[slot];seen=0;waiting=true;sent=now;
        Serial.printf("{\"type\":\"diag_query\",\"round\":%u,\"slot\":%u,\"id\":%u,\"cmd\":%u,\"args\":[",round_no,slot,q.id,q.cmd);
        for(unsigned i=0;i<q.n;++i)Serial.printf("%s%u",i?",":"",q.args[i]);Serial.println("]}");
        if(!transmit(q.id,q.cmd,q.args,q.n)){ready=false;Serial.println("[ReadDiag] TX FAILED");}
    }
    if(!waiting && now-last_query>=250){transmit(255,65,nullptr,0);last_query=now;}
    if(now-last_report>=1000){report();last_report=now;}
    delay(1);
}
