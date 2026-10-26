#include "ADS7128.h"
#include "Wire.h"
#include <cassert>
#include <cstdio>
uint32_t nowMs=0; Console Serial; Bus Wire1;
int main(){
 assert(ads7128.begin());uint16_t raw=999;
 Wire1.regs[0]=8; // stale completion must be cleared
 assert(ads7128.readChannel(5,raw));assert(raw==0x8123);assert(Wire1.triggers==1 && Wire1.blocks==1);
 assert(!ads7128.readChannel(8,raw));assert(raw==0x8123);
 Wire1.timeout=true;auto start=nowMs;assert(!ads7128.readChannel(0,raw));assert(nowMs-start==10);assert(raw==0x8123);assert(Wire1.regs[1]==0);Wire1.timeout=false;
 Wire1.nack=true;assert(!ads7128.begin());assert(!ads7128.readChannel(0,raw));Wire1.nack=false;
 Wire1.shortRead=true;assert(!ads7128.readChannel(0,raw));Wire1.shortRead=false;
 float temps[6]={1,2,3,4,5,6};Wire1.sample=0;assert(!ads7128.readTemperatures(temps));assert(temps[0]==1 && temps[5]==6);
 Wire1.sample=65535;assert(!ads7128.readTemperatures(temps));assert(temps[0]==1);
 Wire1.sample=32768;assert(ads7128.readTemperatures(temps));for(float t:temps)assert(std::fabs(t-25)<0.01);
 nowMs=0xFFFFFFFC;Wire1.timeout=true;assert(!ads7128.readChannel(0,raw));assert(nowMs==6);
 puts("PASS: fresh conversion, adjacent bytes, stale flag, timeout/wraparound, NACK, short read, invalid NTC rails, 25C scan");
}
