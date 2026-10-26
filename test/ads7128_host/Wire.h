#pragma once
#include <stdint.h>
#include <vector>
#include <cassert>
struct Bus {
 uint8_t regs[256]={}; std::vector<uint8_t> tx,rx; unsigned pos=0;
 uint16_t sample=0x8123; bool timeout=false,nack=false,shortRead=false;
 unsigned triggers=0,blocks=0; bool cleared=false; int polls=0;
 void beginTransmission(uint8_t a){assert(a==0x10);tx.clear();}
 void write(uint8_t v){tx.push_back(v);}
 uint8_t endTransmission(bool stop=true){
  if(nack)return 2;
  if(tx[0]==8){assert(tx.size()==3);auto r=tx[1],v=tx[2];
   if(r==0){assert(v==8);regs[0]&=~8;cleared=true;}
   else {if(r==1 && (v&8)){assert(cleared);assert(regs[1]&32);assert(v&32);assert(regs[3]==7);assert(regs[4]==0);assert(regs[16]==0);cleared=false;triggers++;polls=0;auto ch=regs[17];regs[160+2*ch]=sample&255;regs[161+2*ch]=sample>>8;}regs[r]=v;}
  }return 0;
 }
 uint8_t requestFrom(uint8_t a,uint8_t n){
  assert(a==0x10);rx.clear();pos=0;auto r=tx[1];
  if(r==0 && !timeout && ++polls>=2)regs[0]|=8;
  if(n==2){assert(tx[0]==0x30);assert(!(regs[1]&32));blocks++;}
  for(int i=0;i<n-(shortRead?1:0);i++)rx.push_back(regs[r+(tx[0]==0x30?i:0)]);
  return rx.size();
 }
 int available(){return rx.size()-pos;}
 int read(){assert(pos<rx.size());return rx[pos++];}
};
extern Bus Wire1;
