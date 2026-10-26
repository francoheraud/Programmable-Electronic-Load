#pragma once
#include <stdint.h>
#include <stddef.h>
#include <cstring>
#include <cstdio>
#include <cassert>
#include <deque>
#include <vector>
#include <map>
#include <string>
#include <memory>
struct Done {};
extern unsigned clockMs, sdCalls, flushes;
extern int receiveBudget;
extern bool mountOk, shortWrite, openOk;
inline uint32_t millis(){return clockMs;}
struct SerialMock { void println(const char*){} template<class... T> void printf(const char*,T...){} };
extern SerialMock Serial;
struct SPIClass { void begin(int,int,int,int){} };
struct Data { std::string text; bool closed=false; };
extern std::map<std::string,std::shared_ptr<Data>> files;
struct File {
 std::shared_ptr<Data> d;
 operator bool() const {return d && !d->closed;}
 size_t write(const uint8_t* b,size_t n){++sdCalls;if(shortWrite)return 0;d->text.append((const char*)b,n);return n;}
 void flush(){++sdCalls;++flushes;}
 void close(){++sdCalls;d->closed=true;}
 int getWriteError(){return 0;}
};
struct SDMock {
 bool begin(int,SPIClass&,unsigned){++sdCalls;return mountOk;}
 void end(){++sdCalls;}
 bool exists(const char* p){++sdCalls;return files.count(p);}
 File open(const char* p,int){++sdCalls;if(!openOk)return {};auto d=std::make_shared<Data>();files[p]=d;return {d};}
};
extern SDMock SD;
#define FILE_WRITE 1
using BaseType_t=int;
struct Queue {size_t cap,size;std::deque<std::vector<uint8_t>> items;};
using QueueHandle_t=Queue*;
using TaskHandle_t=void*;
#define pdPASS 1
#define pdTRUE 1
#define pdMS_TO_TICKS(x) (x)
inline QueueHandle_t xQueueCreate(size_t cap,size_t size){return new Queue{cap,size,{}};}
inline void vQueueDelete(QueueHandle_t q){delete q;}
inline int xQueueSend(QueueHandle_t q,const void* p,int wait){assert(wait==0);if(q->items.size()==q->cap)return 0;auto b=(const uint8_t*)p;q->items.emplace_back(b,b+q->size);return 1;}
inline int xQueueReceive(QueueHandle_t q,void* p,unsigned timeout){if(receiveBudget--<=0)throw Done{};if(q->items.empty()){clockMs+=timeout;return 0;}memcpy(p,q->items.front().data(),q->size);q->items.pop_front();return 1;}
extern void(*taskFn)(void*);
extern void* taskArg;
inline int xPortGetCoreID(){return 1;}
inline int xTaskCreatePinnedToCore(void(*f)(void*),const char*,int,void* arg,int,TaskHandle_t* h,int core){assert(core==0);taskFn=f;taskArg=arg;*h=arg;return 1;}
inline void vTaskDelay(int){}
