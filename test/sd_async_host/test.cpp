#include <algorithm>
#include "SdLog.h"
unsigned clockMs=0,sdCalls=0,flushes=0;
int receiveBudget=0;
bool mountOk=true,shortWrite=false,openOk=true;
SerialMock Serial; SDMock SD;
std::map<std::string,std::shared_ptr<Data>> files;
void(*taskFn)(void*)=nullptr;void* taskArg=nullptr;
void pump(int n=100){receiveBudget=n;try{taskFn(taskArg);}catch(Done&) {}}
void reset(){files.clear();clockMs=sdCalls=flushes=0;mountOk=true;shortWrite=false;openOk=true;}
int main(){
 SPIClass spi; LogRecord r{};r.state=LoadState::RUNNING_CC;r.voltage=12;r.current=1;
 {reset();SdLog l;assert(l.begin(spi,12,13,11,10));unsigned calls=sdCalls;
  assert(l.startTest());for(int i=0;i<64;i++){r.timestampMs=i*500;assert(l.append(r));}
  assert(!l.append(r));assert(l.dropped()==1);l.endTest();assert(sdCalls==calls);assert(l.busy());
  pump();assert(!l.busy());assert(files.size()==1);assert(files.begin()->second->closed);
  const auto& s=files.begin()->second->text;assert(s.find("sample_no,dropped_total")!=std::string::npos);
  assert(std::count(s.begin(),s.end(),'\n')==65);
 }
 {reset();SdLog l;assert(l.begin(spi,12,13,11,10));assert(l.startTest());r.voltage=11;l.append(r);l.endTest();
  assert(l.startTest());r.voltage=22;l.append(r);l.endTest();pump();assert(files.size()==2);
  assert(files["/test_0001.csv"]->text.find("11.000")!=std::string::npos);
  assert(files["/test_0001.csv"]->text.find("22.000")==std::string::npos);
  assert(files["/test_0002.csv"]->text.find("22.000")!=std::string::npos);assert(!l.busy());
 }
 {reset();SdLog l;assert(l.begin(spi,12,13,11,10));l.startTest();l.append(r);pump(1);unsigned before=flushes;
  pump(21);assert(flushes>before);assert(l.busy());l.endTest();pump(1);assert(!l.busy());
 }
 {reset();SdLog l;assert(l.begin(spi,12,13,11,10));l.startTest();l.append(r);pump(1);
  shortWrite=true;l.append(r);l.endTest();pump();assert(l.errors()==1);assert(!l.ready());assert(!l.busy());assert(!l.startTest());
 }
 {reset();openOk=false;SdLog l;assert(l.begin(spi,12,13,11,10));l.startTest();l.append(r);l.endTest();pump();assert(l.errors()==1);assert(!l.busy());}
 {reset();mountOk=false;SdLog l;assert(!l.begin(spi,12,13,11,10));assert(!l.startTest());assert(!l.append(r));assert(!l.busy());}
 puts("PASS: nonblocking producer, full queue/end, run separation, periodic flush, short write, open failure, missing card");
}
