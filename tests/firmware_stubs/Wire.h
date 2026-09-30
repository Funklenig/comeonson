#pragma once
#include <stdint.h>
struct FakeWire {
  void begin() {}
  void setClock(int) {}
  void setWireTimeout(int,bool) {}
  void beginTransmission(int) {}
  void write(int) {}
  int endTransmission() { return 0; }
  int requestFrom(int,int) { return 0; } // absent sensors: preserve invalid flags/ages
  int available() { return 0; }
  int read() { return 0; }
};
extern FakeWire Wire, Wire1;
