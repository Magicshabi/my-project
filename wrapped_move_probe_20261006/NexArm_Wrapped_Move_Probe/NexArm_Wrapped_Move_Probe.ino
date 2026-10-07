#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "ProtocolSelfTest.h"
#include "RealJointFeedback.h"
#include "RecordJson.h"
#include "TeachCapture.h"
#include "WrappedMovePolicy.h"
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* VERSION="nexarm-wrapped-move-probe-r5";
// Phases: 0 IDLE, 1 BASELINE, 2 UP_READY, 3 UP_OBSERVE, 4 DOWN_READY, 5 DOWN_OBSERVE, 6 COORDS_AFTER, 7 FINISHED
enum Phase : uint8_t { IDLE, BASELINE, UP_READY, UP_OBSERVE, DOWN_READY, DOWN_OBSERVE, COORDS_AFTER, FINISHED };
static Phase phase=IDLE;
static CommProtocol_t rx,tx;
static HandleTeachRecord record;
static TeachCapture sample;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2,U8X8_PIN_NONE,27,26);
static bool ready=false,found=false,spent=false;
static bool up_attempted=false,up_sent=false,down_attempted=false,down_sent=false;
static bool moved=false,returned=false;
static uint32_t last_rx=0,last_actual=0,last_query=0,last_report=0,stage_ms=0,test_start=0,frames=0;
static int16_t current[6]={},base[6]={},end_pos[6]={};
static int16_t max_delta[6]={};
static const char* reason="waiting_host";
static char host_line[40]={};static uint8_t host_used=0;static bool host_overflow=false;

static bool write_frame(uint8_t id,uint8_t cmd,const uint8_t* a,size_t n,WMPermit permit){
    bool stage_ok = permit==WM_NONE ||
        (permit==WM_UP_ONCE && phase==UP_READY && !up_attempted) ||
        (permit==WM_DOWN_RETURN && phase==DOWN_READY && up_sent && !down_attempted);
    if(!stage_ok || !wrapped_tx_allowed(id,cmd,a,n,permit)) return false;
    uint16_t len=tx.tx_packet_complete(id,cmd,a,n);if(!len)return false;
    if(permit==WM_UP_ONCE)up_attempted=true;
    if(permit==WM_DOWN_RETURN)down_attempted=true; // consume before UART; never retry
    bool ok=Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet),len)==len;
    if(permit==WM_UP_ONCE)up_sent=ok;
    if(permit==WM_DOWN_RETURN)down_sent=ok;
    if(permit!=WM_NONE){
        char hex[48];const uint8_t* p=reinterpret_cast<const uint8_t*>(&tx.tx_packet); // frames up to 20B -> 2*20+1 chars
        for(unsigned i=0;i<len;++i)snprintf(hex+i*2,3,"%02x",p[i]);hex[2*len]=0;
        Serial.printf("{\"type\":\"wrapped_probe_tx\",\"permit\":%u,\"uart_complete\":%s,\"wire_hex\":\"%s\",\"ms\":%lu}\n",(unsigned)permit,ok?"true":"false",hex,(unsigned long)millis());
    }
    return ok;
}
static void evidence(){
    Serial.printf("{\"type\":\"wrapped_probe_state\",\"firmware\":\"%s\",\"self_test_passed\":true,\"phase\":%u,\"spent\":%s,\"up_sent\":%s,\"up_uart_complete\":%s,\"down_sent\":%s,\"down_uart_complete\":%s,\"moved\":%s,\"returned\":%s,\"max_delta\":[%d,%d,%d,%d,%d,%d],\"baseline\":[%d,%d,%d,%d,%d,%d],\"final\":[%d,%d,%d,%d,%d,%d],\"frames\":%lu,\"reason\":\"%s\",\"playback_validated\":false}\n",
        VERSION,(unsigned)phase,spent?"true":"false",up_sent?"true":"false",up_sent?"true":"false",down_sent?"true":"false",down_sent?"true":"false",
        moved?"true":"false",returned?"true":"false",
        max_delta[0],max_delta[1],max_delta[2],max_delta[3],max_delta[4],max_delta[5],
        base[0],base[1],base[2],base[3],base[4],base[5],
        end_pos[0],end_pos[1],end_pos[2],end_pos[3],end_pos[4],end_pos[5],
        (unsigned long)frames,reason);
}
static void finish(const char* why){
    reason=why;
    memcpy(end_pos,current,sizeof(current));
    phase=FINISHED;evidence();
}
static void log_wrapped_rx(PacketTypeDef* p){
    char hex[65];unsigned n=p->elements.length>2?p->elements.length-2:0;if(n>32)n=32;
    for(unsigned i=0;i<n;++i)snprintf(hex+i*2,3,"%02x",p->elements.args[i]);hex[2*n]=0;
    Serial.printf("{\"type\":\"wrapped_rx\",\"id\":%u,\"cmd\":%u,\"args_hex\":\"%s\",\"ms\":%lu}\n",
        (unsigned)p->elements.id,(unsigned)p->elements.cmd,hex,(unsigned long)millis());
}
static void track_delta(){
    for(unsigned i=0;i<6;++i){
        int d=current[i]-base[i];if(d<0)d=-d;
        if(d>max_delta[i])max_delta[i]=d;
        if(d>300){finish("motion_outside_envelope_cut_power");return;}
        if(d>4)moved=true;
    }
}
static void receive(PacketTypeDef* p){
    if(p->elements.id==0x5A && p->elements.cmd==11){log_wrapped_rx(p);return;}
    if(p->elements.id!=0x5A || p->elements.cmd!=65 || !decode_real_joint_positions(p->elements.args,p->elements.length-2,current)){
        if(p->elements.id==0x5A)log_wrapped_rx(p);
        return;
    }
    last_actual=millis();
    Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",(unsigned long)last_actual,current[0],current[1],current[2],current[3],current[4],current[5]);
    if(phase==BASELINE && last_actual-stage_ms>=600 && sample.count<5){if(!sample.add(current)||sample.high[5]-sample.low[5]>2)finish("baseline_unstable");}
    if(phase==UP_OBSERVE || phase==DOWN_OBSERVE){++frames;track_delta();}
}
static void start(){
    if(!ready||phase!=IDLE||spent)return;
    spent=true;phase=BASELINE;stage_ms=millis();test_start=stage_ms;sample.reset();reason="baseline_sampling";
    Serial.println("[WrappedProbe] Authorized one-shot wrapped move test starting");
}
static void report(){
    evidence();char line[RECORD_JSON_SIZE];size_t n=format_record_json(record,VERSION,"handle_free",false,line,sizeof(line));if(n)Serial.write((uint8_t*)line,n);
    if(found){oled.firstPage();do{oled.setFont(u8g2_font_6x12_tr);oled.drawStr(0,13,"WRAPPED MOVE r5");oled.drawStr(0,29,"dz +30mm 3000ms");
        oled.drawStr(0,45,phase==IDLE?"Waiting host":phase==FINISHED?(moved?"Moved; see log":"No motion; see log"):"Testing; watch arm");
        oled.drawStr(0,61,"K2:abort observe");}while(oled.nextPage());}
}
void setup(){
    Serial.begin(1000000);delay(300);pinMode(2,INPUT_PULLDOWN);Wire.setPins(26,27);Wire.begin();Wire.setClock(400000);Wire.beginTransmission(0x3C);found=Wire.endTransmission()==0;if(found)oled.begin();
    if(!(wrapped_move_policy_test()&&teach_capture_self_test()&&protocol_boundary_self_test()&&real_joint_feedback_self_test())){Serial.println("[WrappedProbe] SELF TEST FAIL; UART disabled");return;}
    Preferences s;bool ok=s.begin("handle_free",true);if(ok){ok=s.getBytesLength("poses_v1")==sizeof(record)&&s.getBytes("poses_v1",&record,sizeof(record))==sizeof(record)&&valid_handle_record(record)&&record.valid_mask==255;s.end();}if(!ok){Serial.println("[WrappedProbe] INVALID RECORD; UART disabled");return;}
    rx.begin();tx.begin();rx.register_success_callback(receive);Serial1.setRxBufferSize(2048);Serial1.begin(1000000,SERIAL_8N1,17,16);ready=true;
    Serial.println("[WrappedProbe] nexarm-wrapped-move-probe-r5 READY; no automatic start");report();
}
void loop(){
    if(!ready){delay(10);return;}
    for(unsigned i=0;i<2048&&Serial1.available();++i){uint32_t t=millis();if(rx.has_partial_frame()&&t-last_rx>100)rx.discard_partial_frame();last_rx=t;uint8_t b=Serial1.read();rx.parsing(&b,1);}
    for(unsigned i=0;i<128&&Serial.available();++i){char c=Serial.read();if(c=='\n'){
        host_line[host_used]=0;if(!host_overflow&&!strcmp(host_line,"RUN_WRAPPED_ONCE"))start();
        if(!host_overflow&&!strcmp(host_line,"STOP_WRAPPED")&&phase!=IDLE&&phase!=FINISHED)finish("host_stop");host_used=0;host_overflow=false;
    }else if(c!='\r'){if(host_used<sizeof(host_line)-1&&!host_overflow)host_line[host_used++]=c;else host_overflow=true;}}
    uint32_t now=millis();
    if(phase!=IDLE&&phase!=FINISHED&&digitalRead(2)==HIGH)finish("key2_stop");
    if(phase!=IDLE&&phase!=FINISHED&&now-last_actual>700)finish("feedback_stale");
    if(phase!=IDLE&&phase!=FINISHED&&now-test_start>25000)finish("test_deadline");
    if(phase==BASELINE&&now-stage_ms>3500)finish("baseline_timeout");
    if(phase==BASELINE&&sample.count==5){
        bool tight=sample.result(base);if(tight)for(unsigned j=0;j<6;++j)if(sample.high[j]-sample.low[j]>2)tight=false;
        if(!tight)finish("baseline_unstable");
        else{write_frame(255,11,nullptr,0,WM_NONE);phase=UP_READY;stage_ms=millis();reason="coords_logged_before_up";}}
    now=millis();
    if(phase==UP_READY&&now-stage_ms>=500){
        const uint8_t up[14]={0,0,0,0,30,0,0,0,0,0,0,0,0xB8,0x0B};
        if(!write_frame(255,50,up,14,WM_UP_ONCE))finish("up_tx_failed");
        else{phase=UP_OBSERVE;stage_ms=millis();reason="observing_up_step";}}
    now=millis();
    if(phase==UP_OBSERVE&&now-stage_ms>=6000){
        if(!moved)finish("no_wrapped_motion");
        else{phase=DOWN_READY;stage_ms=millis();reason="returning_down";}}
    now=millis();
    if(phase==DOWN_READY){
        const uint8_t down[14]={0,0,0,0,0xE2,0xFF,0,0,0,0,0,0,0xB8,0x0B};
        if(!write_frame(255,50,down,14,WM_DOWN_RETURN))finish("down_tx_failed");
        else{phase=DOWN_OBSERVE;stage_ms=millis();reason="observing_down_step";}}
    now=millis();
    if(phase==DOWN_OBSERVE&&now-stage_ms>=5000){
        returned=true;for(unsigned i=0;i<6;++i){int d=current[i]-base[i];if(d<0)d=-d;if(d>8)returned=false;}
        write_frame(255,11,nullptr,0,WM_NONE);phase=COORDS_AFTER;stage_ms=millis();reason="coords_logged_after";}
    now=millis();
    if(phase==COORDS_AFTER&&now-stage_ms>=600)finish(returned?"wrapped_motion_observed_returned":"wrapped_motion_observed_return_unverified");
    now=millis();if(now-last_query>=100){if(!write_frame(255,65,nullptr,0,WM_NONE)&&phase!=IDLE&&phase!=FINISHED)finish("query_tx_failed");last_query=now;}
    if(now-last_report>=500){report();last_report=now;}delay(1);
}
