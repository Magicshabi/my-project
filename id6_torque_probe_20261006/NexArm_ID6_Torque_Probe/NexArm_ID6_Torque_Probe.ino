#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "ProtocolSelfTest.h"
#include "RealJointFeedback.h"
#include "RecordJson.h"
#include "TeachCapture.h"
#include "TorquePolicy.h"
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* VERSION="nexarm-id6-torque-probe-r3";
enum Phase { IDLE, BASELINE, PRELOADED, HOLDING, STEPPING, FINISHED };
static Phase phase=IDLE;
static CommProtocol_t rx,tx;
static HandleTeachRecord record;
static TeachCapture sample;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2,U8X8_PIN_NONE,27,26);
static bool ready=false,found=false,spent=false,preloaded=false,on_attempted=false,on_sent=false,off_attempted=false,off_sent=false,step_sent=false,followed=false,stable=false;
static uint32_t last_rx=0,last_actual=0,last_query=0,last_report=0,stage_ms=0,on_ms=0,frames=0;
static int16_t current[6]={},base[6]={},end_pos[6]={},goal=-1,recent[5][6]={};
static uint8_t recent_count=0,recent_head=0;
static const char* reason="waiting_host";
static char host_line[40]={};static uint8_t host_used=0;static bool host_overflow=false;

static bool write_frame(uint8_t id,uint8_t cmd,const uint8_t* a,size_t n,TxPermit permit){
    bool stage_ok=permit==NONE || (permit==PRELOAD_CURRENT && phase==BASELINE && !preloaded) ||
        (permit==ENABLE_ID6 && phase==PRELOADED && preloaded && !on_attempted) ||
        (permit==OPEN_ID6 && phase==HOLDING && on_sent && !step_sent) ||
        (permit==RELEASE_ID6 && on_attempted && !off_attempted);
    if(!stage_ok || !torque_tx_allowed(id,cmd,a,n,permit,base[5],goal)) return false;
    uint16_t len=tx.tx_packet_complete(id,cmd,a,n);if(!len)return false;
    if(permit==ENABLE_ID6){on_attempted=true;on_ms=millis();}
    if(permit==RELEASE_ID6)off_attempted=true;
    if(permit==PRELOAD_CURRENT)preloaded=true; // consume before UART; never retry
    if(permit==OPEN_ID6)step_sent=true;
    bool ok=Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet),len)==len;
    if(permit==ENABLE_ID6)on_sent=ok;
    if(permit==RELEASE_ID6)off_sent=ok;
    if(permit!=NONE){
        char hex[29];const uint8_t* p=reinterpret_cast<const uint8_t*>(&tx.tx_packet);
        for(unsigned i=0;i<len;++i)snprintf(hex+i*2,3,"%02x",p[i]);hex[2*len]=0;
        Serial.printf("{\"type\":\"torque_probe_tx\",\"permit\":%u,\"uart_complete\":%s,\"wire_hex\":\"%s\",\"ms\":%lu}\n",(unsigned)permit,ok?"true":"false",hex,(unsigned long)millis());
    }
    return ok;
}
static void evidence(){
    Serial.printf("{\"type\":\"torque_probe_state\",\"firmware\":\"%s\",\"self_test_passed\":true,\"phase\":%u,\"spent\":%s,\"preloaded\":%s,\"enable_attempted\":%s,\"enable_uart_complete\":%s,\"step_sent\":%s,\"release_attempted\":%s,\"release_uart_complete\":%s,\"baseline\":[%d,%d,%d,%d,%d,%d],\"final\":[%d,%d,%d,%d,%d,%d],\"goal\":%d,\"stable\":%s,\"step_observed\":%s,\"frames\":%lu,\"reason\":\"%s\",\"torque_register_readback\":false,\"goal_register_readback\":false,\"playback_validated\":false}\n",
        VERSION,(unsigned)phase,spent?"true":"false",preloaded?"true":"false",on_attempted?"true":"false",on_sent?"true":"false",step_sent?"true":"false",off_attempted?"true":"false",off_sent?"true":"false",
        base[0],base[1],base[2],base[3],base[4],base[5],end_pos[0],end_pos[1],end_pos[2],end_pos[3],end_pos[4],end_pos[5],goal,stable?"true":"false",followed?"true":"false",(unsigned long)frames,reason);
}
static void finish(const char* why){
    reason=why;
    if(on_attempted && !off_attempted){const uint8_t off[]={40,0};if(!write_frame(6,3,off,2,RELEASE_ID6))reason="release_tx_failed_cut_power";}
    if(!stable)memcpy(end_pos,current,sizeof(current));
    phase=FINISHED;evidence();
}
static bool close_to_base(int bound){for(unsigned i=0;i<6;++i)if(abs(current[i]-base[i])>bound)return false;return true;}
static void receive(PacketTypeDef* p){
    if(p->elements.id!=0x5A || p->elements.cmd!=65 || !decode_real_joint_positions(p->elements.args,p->elements.length-2,current))return;
    last_actual=millis();
    Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",(unsigned long)last_actual,current[0],current[1],current[2],current[3],current[4],current[5]);
    if(phase==BASELINE && last_actual-stage_ms>=600 && sample.count<5){if(!sample.add(current)||sample.high[5]-sample.low[5]>2)finish("baseline_unstable");}
    if(phase==PRELOADED || phase==HOLDING){if(!close_to_base(4))finish("hold_position_changed_cut_power");}
    if(phase==STEPPING){
        ++frames;
        for(unsigned i=0;i<5;++i)if(abs(current[i]-base[i])>8){finish("other_joint_changed_cut_power");return;}
        if(current[5]>base[5]+4 || current[5]<base[5]-24){finish("id6_outside_envelope_cut_power");return;}
        if(last_actual-stage_ms>=1500){memcpy(recent[recent_head],current,sizeof(current));recent_head=(recent_head+1)%5;if(recent_count<5)++recent_count;}
    }
}
static bool position(TxPermit permit,int16_t target){uint8_t a[]={41,5,(uint8_t)target,(uint8_t)(target>>8),0,0,30,0};return write_frame(6,3,a,8,permit);}
static void start(){
    if(!ready||phase!=IDLE||spent)return;
    spent=true;phase=BASELINE;stage_ms=millis();sample.reset();reason="baseline_sampling";
    Serial.println("[TorqueProbe] Authorized one-shot ID6 test starting");
}
static void report(){
    evidence();char line[RECORD_JSON_SIZE];size_t n=format_record_json(record,VERSION,"handle_free",false,line,sizeof(line));if(n)Serial.write((uint8_t*)line,n);
    if(found){oled.firstPage();do{oled.setFont(u8g2_font_6x12_tr);oled.drawStr(0,13,"ID6 TORQUE TEST r3");oled.drawStr(0,29,"ONLY GRIPPER ID6");
        oled.drawStr(0,45,phase==IDLE?"Waiting host":phase==FINISHED?(followed?"Step seen":"No pass; see log"):"Testing; watch ID6");
        oled.drawStr(0,61,off_sent?"OFF sent; unverified":"K2:request release");}while(oled.nextPage());}
}
void setup(){
    Serial.begin(1000000);delay(300);pinMode(2,INPUT_PULLDOWN);Wire.setPins(26,27);Wire.begin();Wire.setClock(400000);Wire.beginTransmission(0x3C);found=Wire.endTransmission()==0;if(found)oled.begin();
    if(!(torque_policy_test()&&single_joint_policy_test()&&teach_capture_self_test()&&protocol_boundary_self_test()&&real_joint_feedback_self_test())){Serial.println("[TorqueProbe] SELF TEST FAIL; UART disabled");return;}
    Preferences s;bool ok=s.begin("handle_free",true);if(ok){ok=s.getBytesLength("poses_v1")==sizeof(record)&&s.getBytes("poses_v1",&record,sizeof(record))==sizeof(record)&&valid_handle_record(record)&&record.valid_mask==255;s.end();}if(!ok){Serial.println("[TorqueProbe] INVALID RECORD; UART disabled");return;}
    rx.begin();tx.begin();rx.register_success_callback(receive);Serial1.setRxBufferSize(2048);Serial1.begin(1000000,SERIAL_8N1,17,16);ready=true;
    Serial.println("[TorqueProbe] nexarm-id6-torque-probe-r3 READY; no automatic start");report();
}
void loop(){
    if(!ready){delay(10);return;}
    for(unsigned i=0;i<2048&&Serial1.available();++i){uint32_t t=millis();if(rx.has_partial_frame()&&t-last_rx>100)rx.discard_partial_frame();last_rx=t;uint8_t b=Serial1.read();rx.parsing(&b,1);}
    for(unsigned i=0;i<128&&Serial.available();++i){char c=Serial.read();if(c=='\n'){
        host_line[host_used]=0;if(!host_overflow&&!strcmp(host_line,"RUN_ID6_ONCE"))start();
        if(!host_overflow&&!strcmp(host_line,"STOP_ID6")&&phase!=IDLE&&phase!=FINISHED)finish("host_stop");host_used=0;host_overflow=false;
    }else if(c!='\r'){if(host_used<sizeof(host_line)-1&&!host_overflow)host_line[host_used++]=c;else host_overflow=true;}}
    uint32_t now=millis();
    if(phase!=IDLE&&phase!=FINISHED&&digitalRead(2)==HIGH)finish("key2_stop");
    if(phase!=IDLE&&phase!=FINISHED&&now-last_actual>500)finish("feedback_stale_cut_power");
    if(on_attempted&&!off_attempted&&now-on_ms>6000)finish("torque_deadline_cut_power");
    if(phase==BASELINE&&now-stage_ms>3500)finish("baseline_timeout");
    if(phase==BASELINE&&sample.count==5){uint8_t unused[8];
        if(!sample.result(base)||!make_open_probe(record,base,goal,unused)||!close_to_base(2))finish("baseline_or_taught_range_invalid");
        else if(!position(PRELOAD_CURRENT,base[5]))finish("preload_tx_failed");
        else{phase=PRELOADED;stage_ms=millis();reason="preloaded_current_wait";}}
    now=millis();
    if(phase==PRELOADED&&now-stage_ms>=700){const uint8_t on[]={40,1};
        if(!close_to_base(2))finish("moved_before_enable");
        else if(!write_frame(6,3,on,2,ENABLE_ID6))finish("enable_tx_failed");
        else{phase=HOLDING;stage_ms=millis();reason="holding_current";}}
    now=millis();
    if(phase==HOLDING&&now-stage_ms>=800){
        if(!close_to_base(4))finish("hold_not_stable");
        else if(!position(OPEN_ID6,goal))finish("step_tx_failed");
        else{phase=STEPPING;stage_ms=millis();reason="observing_open_step";}}
    now=millis();
    if(phase==STEPPING&&now-stage_ms>=4000){TeachCapture tail;for(unsigned i=0;i<recent_count;++i)tail.add(recent[i]);stable=tail.result(end_pos)&&tail.high[5]-tail.low[5]<=2;
        followed=stable&&frames>=15&&abs(end_pos[5]-goal)<=3&&end_pos[5]-base[5]<=-4;finish(followed?"small_step_observed":"target_not_verified");}
    now=millis();if(now-last_query>=100){if(!write_frame(255,65,nullptr,0,NONE)&&phase!=IDLE&&phase!=FINISHED)finish("query_tx_failed");last_query=now;}
    if(now-last_report>=500){report();last_report=now;}delay(1);
}
