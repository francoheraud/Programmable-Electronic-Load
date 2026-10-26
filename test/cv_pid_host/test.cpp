#include "CvPid.h"
#include <assert.h>
#include <stdio.h>

static bool near(float a, float b) { return fabsf(a-b)<0.00001f; }
int main() {
  CvPid pid;
  // Enable waits for a new sample; 200ms gaps still respect the slew limit.
  assert(pid.update(true,true,0,0,1,5,4,.2f)==0);
  float u=pid.update(true,true,200,200,2,5,4,.2f);
  assert(near(u,.003f));
  for (unsigned t=220;t<400;t+=20)
    assert(near(pid.update(true,true,t,200,2,5,4,.2f),u));
  float next=pid.update(true,true,400,400,3,5,4,.2f);
  assert(next>u && next-u<=.00301f);
  // Sustained error saturates; reversal unwinds immediately (no accumulated windup).
  unsigned seq=3,t=400;
  for(int i=0;i<1000;i++) { t+=200; u=pid.update(true,true,t,t,++seq,5,4,.2f); }
  assert(near(u,.2f));
  t+=200; next=pid.update(true,true,t,t,++seq,3,4,.2f);
  assert(next<u && u-next<=.00601f);
  // A reduced ceiling overrides even the downward slew limit without a new read.
  assert(near(pid.update(true,true,t+20,t,seq,3,4,.01f),.01f));
  // Stop, invalid readings, stale data and long update gaps always zero output.
  assert(pid.update(false,true,t+40,t,seq,3,4,.2f)==0);
  assert(pid.update(true,true,t+60,t,seq,5,4,.2f)==0);
  t+=260; assert(pid.update(true,true,t,t,++seq,5,4,.2f)>0);
  assert(pid.update(true,false,t+20,t,seq,5,4,.2f)==0);
  assert(pid.update(true,true,t+800,t,seq,5,4,.2f)==0);
  assert(pid.update(true,true,t+800,t+800,++seq,NAN,4,.2f)==0);
  assert(pid.update(true,true,t+800,t+800,++seq,5,4,NAN)==0);
  assert(pid.update(true,true,t+800,t+800,++seq,5,4,0)==0);
  // Millis wraparound is handled by unsigned subtraction.
  pid.reset();
  const uint32_t start=0xFFFFFF80u;
  assert(pid.update(true,true,start,start,1,5,4,.2f)==0);
  assert(near(pid.update(true,true,start+200,start+200,2,5,4,.2f),.003f));
  assert(pid.update(true,true,start+1000,start+1000,3,5,4,.2f)==0);
  // Derivative is active on measurement, but has no setpoint kick.
  CvPidSettings cfg; cfg.kp=0; cfg.ki=0; cfg.kd=.001f; cfg.riseRate=1; cfg.fallRate=1;
  CvPid derivativeOnly(cfg);
  derivativeOnly.update(true,true,0,0,1,4,4,.2f);
  assert(derivativeOnly.update(true,true,200,200,2,4,4,.2f)==0);
  assert(derivativeOnly.update(true,true,400,400,3,4,3,.2f)==0);
  assert(derivativeOnly.update(true,true,600,600,4,5,3,.2f)>0);
  // Same elapsed time with different sample periods gives the same integral action.
  cfg.kp=cfg.kd=0; cfg.ki=.02f;
  CvPid a(cfg),b(cfg);
  a.update(true,true,0,0,0,5,4,1); b.update(true,true,0,0,0,5,4,1);
  float ua=0,ub=0;
  for(unsigned j=1;j<=10;j++) ua=a.update(true,true,j*100,j*100,j,5,4,1);
  for(unsigned j=1;j<=5;j++) ub=b.update(true,true,j*200,j*200,j,5,4,1);
  assert(near(ua,.02f) && near(ua,ub));
  // V3 cadence: no repeated integration and <=0.75mA rise per 50ms.
  CvPid tuned;
  tuned.update(true,true,0,0,0,5,4,.2f);
  float previous=0;
  for(unsigned j=1;j<=400;j++) {
    const float actual=tuned.update(true,true,j*50,j*50,j,5,4,.2f);
    assert(actual>=previous && actual-previous<=.000751f);
    assert(near(tuned.update(true,true,j*50+20,j*50,j,5,4,.2f),actual));
    previous=actual;
  }
  assert(previous>.15f && previous<.17f); // 20 seconds of fixed 1 V error.
  const float reduced=tuned.update(true,true,20050,20050,401,3,4,.2f);
  assert(reduced<previous && previous-reduced<=.001501f);
  assert(tuned.update(false,true,20070,20050,401,3,4,.2f)==0);
  // Compare only command ramp timing, not hardware stability or settling time.
  CvPidSettings baseline; baseline.ki=.005f; baseline.riseRate=.010f;
  CvPid old(baseline), trial;
  old.update(true,true,0,0,0,5,4,.2f); trial.update(true,true,0,0,0,5,4,.2f);
  unsigned oldCross=0,trialCross=0;
  for(unsigned j=1;j<=600;j++) {
    const auto stamp=j*50;
    if(old.update(true,true,stamp,stamp,j,5,4,.2f)>=.09f && !oldCross) oldCross=stamp;
    if(trial.update(true,true,stamp,stamp,j,5,4,.2f)>=.09f && !trialCross) trialCross=stamp;
  }
  assert(trialCross>10000 && trialCross<13000 && oldCross>17000 && oldCross<19000);
  printf("Fixed-error ramp to 0.09 A: v2 %u ms; v3 %u ms (not a plant simulation)\n",oldCross,trialCross);
  puts("PASS: sign, time scaling, freshness, bounds, slew, anti-windup, stop/failure reset, timer wrap, filtered derivative");
}
