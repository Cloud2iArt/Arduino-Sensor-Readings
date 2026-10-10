#pragma once
#include "Arduino.h"
struct FakeWire {
  bool present[128] = {};
  bool shortRead[128] = {};
  uint16_t raw[128] = {};
  int configs[128] = {};
  byte address=0, writes=0, reg=0, readIndex=0;
  void begin() {}
  void setClock(unsigned long) {}
  void setWireTimeout(unsigned long, bool) {}
  void beginTransmission(byte a) { address=a; writes=0; }
  void write(byte value) { if (writes++ == 0) reg=value; }
  int endTransmission(bool = true) {
    if (writes == 3 && reg == 0) ++configs[address];
    return present[address] ? 0 : 2;
  }
  byte requestFrom(byte a, byte) { address=a; readIndex=0; return shortRead[a] ? 1 : 2; }
  int read() { return readIndex++ == 0 ? raw[address] >> 8 : raw[address] & 255; }
};
inline FakeWire Wire;
