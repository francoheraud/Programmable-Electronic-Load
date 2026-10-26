#include "ExternalNtc.h"
#include <assert.h>
#include <stdio.h>
#include <initializer_list>
int main() {
  float t=0;
  assert(externalNtcCelsius(825,t) && fabsf(t-25)<.01f);
  for(int actual=-15;actual<=120;actual+=5) {
    const float r=10000*expf(4100*(1.0f/(actual+273.15f)-1.0f/298.15f));
    const int16_t code=(int16_t)lroundf((3.3f*r/(10000+r))/.002f);
    assert(externalNtcCelsius(code,t));
    assert(fabsf(t-actual)<.5f);
  }
  for(int code : {-1,0,10,1625,1650,2047}) {
    t=123;
    assert(!externalNtcCelsius(code,t) && t==123);
  }
  assert(externalNtcCelsius(1000,t) && t<25);
  assert(externalNtcCelsius(500,t) && t>25);
  puts("PASS: RN3440 nominal curve, ADC scale, open/short/range rejection and output preservation");
}
