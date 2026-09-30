#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string>
#include <algorithm>
constexpr int A0=0, A1=1, SDA=18, SCL=19, WIRE1_SDA_PIN=27, WIRE1_SCL_PIN=26;
constexpr int INPUT_PULLUP=0, INPUT=1, OUTPUT=2, LOW=0, HIGH=1;
constexpr float PI=3.14159265358979323846f;
extern uint64_t simulated_us;
extern int applied_dac, raw_adc;
inline uint32_t micros() { return simulated_us; }
inline uint32_t millis() { return simulated_us/1000; }
inline void delay(int ms) { simulated_us += ms*1000; }
inline void delayMicroseconds(int us) { simulated_us += us; }
inline void pinMode(int,int) {}
inline void digitalWrite(int,int) {}
inline int digitalRead(int) { return HIGH; }
inline void analogWriteResolution(int) {}
inline void analogReadResolution(int) {}
inline void analogWrite(int, int value) { applied_dac=value; }
inline int analogRead(int) { raw_adc = 1400+(millis()%100); return raw_adc; }
template<class T> T constrain(T value, T lo, T hi) { return std::max(lo,std::min(value,hi)); }
struct FakeSerial {
  std::string input, output;
  int capacity=1024;
  void begin(int) {}
  int available() { return input.size(); }
  int read() { char c=input.front(); input.erase(0,1); return c; }
  int availableForWrite() { return capacity; }
  size_t write(const uint8_t *bytes, size_t n) { output.append((const char*)bytes,n); return n; }
};
extern FakeSerial Serial;
