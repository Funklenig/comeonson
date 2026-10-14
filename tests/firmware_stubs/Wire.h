#pragma once
#include <stdint.h>
#include <array>
#include <vector>
#include <Arduino.h>

// Mode 0 absent, 1 valid, 2 bad CRC, 3 short read, 4 NACK,
// 5 zero scale, 6 negative scale. Channels are independent on the mux bus.
struct FakeWire {
  std::array<int,8> modes{};
  std::array<int,8> starts{}, stops{};
  std::vector<uint8_t> tx, rx;
  int address=0, channel=0, offset=0, latency_us=0;
  int flow_raw=33400;
  void begin() {}
  void setClock(int) {}
  void setWireTimeout(int,bool) {}
  void beginTransmission(int addr) { address=addr; tx.clear(); }
  void write(int value) { tx.push_back(value); }
  int endTransmission() {
    simulated_us += latency_us;
    if (address==0x70 && !tx.empty()) {
      for (int ch=0;ch<8;++ch) if (tx[0] == (1<<ch)) channel=ch;
    }
    if (modes[channel]==4) return 2;
    if (address!=0x70 && tx.size()==2) {
      if (tx[0]==0x10 || tx[0]==0x36) ++starts[channel];
      if (tx[0]==0x3f) ++stops[channel];
    }
    return 0;
  }
  void word(uint16_t value, uint8_t initial) {
    const uint8_t hi=value>>8, lo=value;
    uint8_t crc=initial;
    for (uint8_t byte : {hi,lo}) {
      crc ^= byte;
      for (int i=0;i<8;++i) crc=crc&0x80 ? (crc<<1)^0x31 : crc<<1;
    }
    rx.push_back(hi); rx.push_back(lo); rx.push_back(crc);
  }
  int requestFrom(int addr,int) {
    simulated_us += latency_us;
    rx.clear(); offset=0;
    const int mode=modes[channel];
    if (mode==0 || mode==4) return 0;
    if (addr==0x40) word(flow_raw,0);
    else {
      word((channel+1)*60,0xff);
      word(4000,0xff);
      word(mode==5 ? 0 : mode==6 ? (uint16_t)-60 : 60,0xff);
    }
    if (mode==2) rx[2]^=1;
    if (mode==3) rx.pop_back();
    return rx.size();
  }
  int available() { return (int)rx.size()-offset; }
  int read() { return available() ? rx[offset++] : -1; }
};
extern FakeWire Wire, Wire1;
