#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <Preferences.h>
#include "CommProtocol.h"
#include "ProtocolSelfTest.h"
#include "RealJointFeedback.h"
#include "TeachCapture.h"
#include "TeachKeys.h"
#include "SingleJointPolicy.h"

// One physical K1 release may issue ONE ID6 goal, baseline - 8 encoder ticks.
// No torque (ON or OFF), mode, ID, home, calibration, NVS write, or host control.
// A goal already sent cannot be retracted: K2 cancels before TX / marks an abort
// after TX. Physical servo-power disconnection remains the stop for anomalies.
SET_LOOP_TASK_STACK_SIZE(16384);
static const char* VERSION = "nexarm-single-joint-probe-r1";
enum Phase { IDLE, SAMPLING, OBSERVING, DONE, FAULT };
static Phase phase = IDLE;
static CommProtocol_t rx, tx;
static TeachKeys keys;
static TeachCapture samples, tail;
static HandleTeachRecord record;
static U8G2_SSD1306_128X64_NONAME_2_HW_I2C oled(U8G2_R2,U8X8_PIN_NONE,27,26);
static bool ready = false, oled_found = false, spent = false, goal_ack = false;
static uint8_t mask = 0;
static int16_t current[6] = {}, baseline[6] = {}, goal = -1;
static uint32_t actual_ms = 0, started_ms = 0, sent_ms = 0, last_query_ms = 0;
static uint32_t last_rx_ms = 0, last_oled_ms = 0, last_record_ms = 0, last_keys_ms = 0;
static uint32_t observed_frames = 0;
static int16_t recent[5][6] = {};
static uint8_t recent_count = 0, recent_head = 0;
static const char* message = "K1:one open step";

static void record_out() {
    char line[RECORD_JSON_SIZE];
    size_t n = format_record_json(record,VERSION,"handle_free",false,line,sizeof(line));
    if (n) Serial.write(reinterpret_cast<const uint8_t*>(line),n);
}
static void screen() {
    if (!oled_found) return;
    oled.firstPage(); do {
        oled.setFont(u8g2_font_6x12_tr);
        oled.drawStr(0,13,"ID6 OPEN -8 ONLY");
        oled.drawStr(0,29,"NO TORQUE ENABLE");
        oled.drawStr(0,45,message);
        oled.drawStr(0,61,spent ? "Used; no retry" : "K2:cancel before TX");
    } while (oled.nextPage());
}
static void fault(const char* reason) {
    phase = FAULT; message = "STOP: servo power";
    Serial.printf("{\"type\":\"probe_fault\",\"reason\":\"%s\",\"goal_sent\":%s,\"software_stop_sent\":false}\n",reason,spent ? "true":"false");
}
static bool transmit(uint8_t id,uint8_t cmd,const uint8_t* args,size_t n) {
    bool one_write = phase == SAMPLING && !spent;
    if (!allowed_single_tx(id,cmd,args,n,one_write,goal)) { fault("TX whitelist rejected"); return false; }
    uint16_t length = tx.tx_packet_complete(id,cmd,args,n);
    if (!length) { fault("packet encode failed"); return false; }
    // Consume before touching UART: short/failed writes must never be retried.
    if (id == TEST_ID) { spent = true; sent_ms = millis(); }
    size_t sent = Serial1.write(reinterpret_cast<const uint8_t*>(&tx.tx_packet),length);
    if (sent != length) { fault("short UART write; no retry"); return false; }
    return true;
}
static void observe() {
    ++observed_frames;
    for (uint8_t j=0;j<5;++j) if (abs(current[j]-baseline[j]) > 8) { fault("non-test joint changed; cut servo power"); return; }
    if (current[5] > baseline[5]+4 || current[5] < baseline[5]-24) { fault("ID6 outside small-step envelope; cut servo power"); return; }
    if (millis()-sent_ms >= 1500) {
        memcpy(recent[recent_head],current,sizeof(current));
        recent_head=(recent_head+1)%5;
        if(recent_count<5) ++recent_count;
    }
}
static void receive(PacketTypeDef* p) {
    if (p->elements.id == TEST_ID && p->elements.cmd == 0 && p->elements.length == 2 && phase == OBSERVING) {
        goal_ack = true; Serial.println("[SingleProbe] Raw status ACK observed; not a goal readback");
    }
    if (p->elements.id != 0x5A || p->elements.cmd != 65 ||
        !decode_real_joint_positions(p->elements.args,p->elements.length-2,current)) return;
    actual_ms = millis();
    Serial.printf("{\"type\":\"actual65\",\"ms\":%lu,\"pulses\":[%d,%d,%d,%d,%d,%d]}\n",static_cast<unsigned long>(actual_ms),current[0],current[1],current[2],current[3],current[4],current[5]);
    if (phase == SAMPLING && actual_ms-started_ms >= 600 && samples.count < 5) {
        if (!samples.add(current) || samples.high[5]-samples.low[5] > 2) fault("baseline not still");
    } else if (phase == OBSERVING) observe();
}
static void start() {
    if (spent || phase != IDLE) return;
    samples.reset(); tail.reset(); goal_ack = false; observed_frames = 0;
    recent_count=recent_head=0;
    phase = SAMPLING; started_ms = millis(); message = "Hold still; ID6 only";
    Serial.println("[SingleProbe] K1 requested one ID6 open step; stabilizing");
}
static void send_goal() {
    uint8_t args[8];
    if (!samples.result(baseline) || !make_open_probe(record,baseline,goal,args)) { fault("record direction/range or baseline invalid"); return; }
    if (millis()-actual_ms > 350) { fault("feedback stale before TX"); return; }
    for (uint8_t j=0;j<6;++j) if (abs(current[j]-baseline[j]) > 2) { fault("moved before TX"); return; }
    if (!transmit(TEST_ID,3,args,sizeof(args))) return;
    phase = OBSERVING; message = "One goal sent";
    Serial.printf("{\"type\":\"single_goal_sent\",\"id\":6,\"baseline\":%d,\"goal\":%d,\"delta\":-8,\"acc\":5,\"speed\":30,\"torque_command_sent\":false}\n",baseline[5],goal);
}
static void finish() {
    int16_t end[6];
    tail.reset();
    for(uint8_t i=0;i<recent_count;++i) tail.add(recent[i]);
    bool stable = tail.result(end) && tail.high[5]-tail.low[5] <= 2;
    int delta = current[5]-baseline[5];
    bool followed = stable && observed_frames >= 15 && abs(end[5]-goal)<=3 && end[5]-baseline[5]<=-4;
    phase = DONE;
    message = followed ? "Step seen; inspect" : "No proof; NO RETRY";
    Serial.printf("{\"type\":\"single_probe_result\",\"movement_observed\":%s,\"stable\":%s,\"actual_delta\":%d,\"frames\":%lu,\"ack_seen\":%s,\"goal_register_readback\":false,\"playback_validated\":false}\n",
        followed?"true":"false",stable?"true":"false",delta,static_cast<unsigned long>(observed_frames),goal_ack?"true":"false");
}
void setup() {
    Serial.begin(1000000); delay(300);
    pinMode(0,INPUT_PULLUP); pinMode(2,INPUT_PULLDOWN);
    Wire.setPins(26,27); Wire.begin(); Wire.setClock(400000);
    Wire.beginTransmission(0x3C); oled_found=Wire.endTransmission()==0; if(oled_found) oled.begin();
    bool tests=single_joint_policy_test() && teach_capture_self_test() && protocol_boundary_self_test() && real_joint_feedback_self_test() && teach_pin_levels_self_test();
    if(!tests) { message="SELF TEST FAILED"; Serial.println("[SingleProbeTest] FAIL; UART disabled"); screen(); return; }
    Serial.println("[SingleProbeTest] PASS: policy, command whitelist, records, capture, protocol, keys");
    Preferences storage;
    bool saved=storage.begin("handle_free",true);
    if(saved) { saved=storage.getBytesLength("poses_v1")==sizeof(record) && storage.getBytes("poses_v1",&record,sizeof(record))==sizeof(record) && valid_handle_record(record) && record.valid_mask==255; storage.end(); }
    if(!saved) { message="Need 8 valid points"; Serial.println("[SingleProbe] Missing record; UART disabled"); screen(); return; }
    rx.begin(); tx.begin(); rx.register_success_callback(receive);
    Serial1.setRxBufferSize(2048); Serial1.begin(1000000,SERIAL_8N1,17,16);
    ready=true; Serial.printf("[SingleProbe] %s READY; K1 one goal; no torque enable\n",VERSION);
    record_out(); screen();
}
void loop() {
    if(!ready) { delay(10); return; }
    for(unsigned n=0;n<2048 && Serial1.available();++n) {
        uint32_t now=millis(); if(rx.has_partial_frame() && now-last_rx_ms>100) rx.discard_partial_frame();
        last_rx_ms=now; uint8_t b=Serial1.read(); rx.parsing(&b,1);
    }
    for(unsigned n=0;n<256 && Serial.available();++n) Serial.read();
    uint32_t now=millis(); if(rx.has_partial_frame() && now-last_rx_ms>100) rx.discard_partial_frame();
    mask=teach_pressed_mask(digitalRead(0),digitalRead(2)); uint8_t click=keys.update(mask,now);
    if(click==2 || (phase==SAMPLING && mask&2)) { phase=spent?DONE:IDLE; message=spent?"TX cannot retract":"Canceled before TX"; }
    // K2 cannot retract the servo's received target; never promise a motor stop.
    if(click==2 && spent) { phase=FAULT; message="STOP: servo power"; Serial.println("[SingleProbe] K2 after TX: disconnect servo power if motion is abnormal"); }
    if(click==1) start();
    now=millis();
    if(phase==SAMPLING && now-started_ms>3500) fault("baseline timeout");
    if(phase==SAMPLING && samples.count==5) send_goal();
    now=millis();
    if(phase==OBSERVING && now-actual_ms>500) fault("feedback lost after goal; cut servo power");
    if(phase==OBSERVING && now-sent_ms>=4000) finish();
    if(now-last_query_ms>=125) { transmit(255,65,nullptr,0); last_query_ms=now; }
    if(now-last_oled_ms>=200) { screen(); last_oled_ms=now; }
    if(now-last_record_ms>=5000) { record_out(); last_record_ms=now; }
    if(now-last_keys_ms>=1000) { Serial.printf("{\"type\":\"single_probe_state\",\"phase\":%u,\"spent\":%s,\"key_mask\":%u}\n",static_cast<unsigned>(phase),spent?"true":"false",mask); last_keys_ms=now; }
    delay(1);
}
