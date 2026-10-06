#pragma once
#include "SingleJointPolicy.h"
enum TxPermit { NONE, PRELOAD_CURRENT, ENABLE_ID6, OPEN_ID6, RELEASE_ID6 };
inline bool torque_tx_allowed(uint8_t id,uint8_t cmd,const uint8_t* a,size_t n,TxPermit permit,int16_t baseline,int16_t goal){
    if(id==255 && cmd==65 && n==0) return true;
    if(id!=6 || cmd!=3 || !a) return false;
    if(permit==ENABLE_ID6 || permit==RELEASE_ID6)
        return n==2 && a[0]==40 && a[1]==(permit==ENABLE_ID6?1:0);
    if(baseline<0 || baseline>4095 || goal!=baseline-8 || goal<0) return false;
    int16_t target=permit==PRELOAD_CURRENT?baseline:goal;
    return (permit==PRELOAD_CURRENT || permit==OPEN_ID6) && n==8 && a[0]==41 && a[1]==5 &&
        a[2]==(target&255) && a[3]==(target>>8) && a[4]==0 && a[5]==0 && a[6]==30 && a[7]==0;
}
inline bool torque_policy_test(){
    uint8_t on[]={40,1},off[]={40,0},hold[]={41,5,0x78,0x05,0,0,30,0},open[]={41,5,0x70,0x05,0,0,30,0};
    for(unsigned id=0;id<256;++id) for(unsigned cmd=0;cmd<256;++cmd){
        if(torque_tx_allowed(id,cmd,on,2,ENABLE_ID6,1400,1392)!=(id==6 && cmd==3)) return false;
        if(torque_tx_allowed(id,cmd,off,2,RELEASE_ID6,1400,1392)!=(id==6 && cmd==3)) return false;
    }
    for(int p=NONE;p<=RELEASE_ID6;++p){
        if(torque_tx_allowed(6,3,on,2,(TxPermit)p,1400,1392)!=(p==ENABLE_ID6) ||
           torque_tx_allowed(6,3,off,2,(TxPermit)p,1400,1392)!=(p==RELEASE_ID6) ||
           torque_tx_allowed(6,3,hold,8,(TxPermit)p,1400,1392)!=(p==PRELOAD_CURRENT) ||
           torque_tx_allowed(6,3,open,8,(TxPermit)p,1400,1392)!=(p==OPEN_ID6)) return false;
    }
    for(unsigned i=0;i<8;++i){open[i]^=1;if(torque_tx_allowed(6,3,open,8,OPEN_ID6,1400,1392))return false;open[i]^=1;}
    return !torque_tx_allowed(6,3,nullptr,2,ENABLE_ID6,1400,1392) &&
        !torque_tx_allowed(6,3,open,8,OPEN_ID6,1400,1391) && !torque_tx_allowed(6,3,on,1,ENABLE_ID6,1400,1392);
}
