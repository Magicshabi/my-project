#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
struct Query { uint8_t id,cmd,n,args[2]; };
static const Query queries[]={
    {255,94,0,{0,0}}, // AT32_OTA::query_version, not an OTA/write command
    {255,11,0,{0,0}}, // factory update_status; compare reported pose with CMD65
    {255,75,1,{6,0}}, // factory read_servo_max_torque
    {255,77,1,{6,0}}, // factory read_servo_angle_limit
    {255,73,1,{6,0}}, // factory read_servo_baud
    {255,71,1,{6,0}}, // factory read_servo_overload (configured protection, not live load)
    {6,1,0,{0,0}},    // raw servo PING
    {6,2,2,{40,1}}, {6,2,2,{42,2}}, {6,2,2,{33,1}}, {6,2,2,{56,2}}
};
static constexpr size_t QUERY_COUNT=sizeof(queries)/sizeof(queries[0]);
inline bool read_allowed(uint8_t id,uint8_t cmd,const uint8_t* a,size_t n){
    if(id==255 && cmd==65 && n==0) return true;
    for(const auto& q:queries) if(id==q.id && cmd==q.cmd && n==q.n && (!n || (a && !memcmp(a,q.args,n)))) return true;
    return false;
}
inline bool read_policy_test(){
    for(const auto& q:queries) if(!read_allowed(q.id,q.cmd,q.args,q.n)) return false;
    for(unsigned id=0;id<256;++id) for(unsigned cmd=0;cmd<256;++cmd){
        uint8_t a[2]={40,1};
        bool expected=(id==6 && cmd==2);
        if(read_allowed(id,cmd,a,2)!=expected) return false;
        bool empty=(id==255 && (cmd==65 || cmd==94 || cmd==11)) || (id==6 && cmd==1);
        if(read_allowed(id,cmd,nullptr,0)!=empty) return false;
    }
    for(unsigned value=0;value<256;++value){
        uint8_t a[]={static_cast<uint8_t>(value)};
        if(read_allowed(255,75,a,1)!=(value==6)) return false;
    }
    return !read_allowed(6,2,nullptr,2) && !read_allowed(255,75,nullptr,1);
}
