#ifndef TRIAL_EVIDENCE_H
#define TRIAL_EVIDENCE_H
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

// RAM only: a finished trial is re-emitted until reset. Never replay a command.
struct TrialEvidence {
    bool available=false, goal_attempted=false, goal_sent=false, baseline_valid=false;
    bool stable=false, matched=false, ack=false;
    uint32_t frames=0;
    int16_t baseline[6]={}, final_pos[6]={}, goal=-1;
    char wire[29]={};  // 14-byte goal frame, hex plus terminator
    const char* reason="pending";
};
inline const char* trial_reason(bool stable, uint32_t frames, int16_t baseline,
                                int16_t final_pos, int16_t goal) {
    if(frames<15) return "insufficient_feedback";
    if(!stable) return "unstable_feedback";
    int delta=final_pos-baseline, error=final_pos-goal;
    if(error>=-3 && error<=3 && delta<=-4) return "small_step_observed";
    if(delta>=-3 && delta<=3) return "no_resolved_motion";
    return "target_not_matched";
}
inline size_t format_trial_evidence(const TrialEvidence& e,char* out,size_t capacity) {
    if(!out || !capacity) return 0;
    int n=snprintf(out,capacity,
        "{\"type\":\"single_probe_evidence\",\"firmware\":\"nexarm-single-joint-probe-r1b\","
        "\"available\":%s,\"goal_attempted\":%s,\"goal_sent\":%s,\"baseline_valid\":%s,"
        "\"baseline\":[%d,%d,%d,%d,%d,%d],\"final\":[%d,%d,%d,%d,%d,%d],"
        "\"goal\":%d,\"wire_hex\":\"%s\",\"stable\":%s,\"movement_observed\":%s,"
        "\"frames\":%lu,\"ack_seen\":%s,\"reason\":\"%s\","
        "\"goal_register_readback\":false,\"torque_command_sent\":false,"
        "\"playback_validated\":false,\"survives_reset\":false}\n",
        e.available?"true":"false",e.goal_attempted?"true":"false",e.goal_sent?"true":"false",e.baseline_valid?"true":"false",
        e.baseline[0],e.baseline[1],e.baseline[2],e.baseline[3],e.baseline[4],e.baseline[5],
        e.final_pos[0],e.final_pos[1],e.final_pos[2],e.final_pos[3],e.final_pos[4],e.final_pos[5],
        e.goal,e.wire,e.stable?"true":"false",e.matched?"true":"false",
        static_cast<unsigned long>(e.frames),e.ack?"true":"false",e.reason);
    if(n<0 || static_cast<size_t>(n)>=capacity) { out[0]=0; return 0; }
    return static_cast<size_t>(n);
}
inline bool trial_evidence_self_test() {
    if(strcmp(trial_reason(true,20,1400,1392,1392),"small_step_observed") ||
       strcmp(trial_reason(true,20,1400,1400,1392),"no_resolved_motion") ||
       strcmp(trial_reason(true,20,1400,1396,1392),"target_not_matched") ||
       strcmp(trial_reason(false,20,1400,1392,1392),"unstable_feedback") ||
       strcmp(trial_reason(true,2,1400,1392,1392),"insufficient_feedback")) return false;
    TrialEvidence e; e.available=e.goal_attempted=e.goal_sent=e.baseline_valid=true;
    e.baseline[5]=1400; e.final_pos[5]=1400; e.goal=1392;
    e.reason="no_resolved_motion"; e.frames=32;
    char line[1024],again[1024],small[8];
    size_t n=format_trial_evidence(e,line,sizeof(line));
    return n && n==format_trial_evidence(e,again,sizeof(again)) && !strcmp(line,again) &&
        strstr(line,"\"reason\":\"no_resolved_motion\"") &&
        strstr(line,"\"goal\":1392") && strstr(line,"\"survives_reset\":false") &&
        !format_trial_evidence(e,small,sizeof(small)) && small[0]==0 &&
        !format_trial_evidence(e,nullptr,0);
}
#endif
