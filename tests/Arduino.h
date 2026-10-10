#pragma once
#include <cstdint>
#include <sstream>
#include <iomanip>
using byte = uint8_t;
constexpr byte A0=14, A1=15, INPUT=0, OUTPUT=1, INPUT_PULLUP=2, LOW=0, HIGH=1, FALLING=2;
#define F(s) s
inline unsigned long fakeNow=0;
inline unsigned long millis() { return fakeNow; }
inline unsigned long micros() { return fakeNow*1000; }
inline void pinMode(byte, byte) {}
inline void digitalWrite(byte, byte) {}
inline int digitalRead(byte) { return HIGH; }
inline int analogRead(byte pin) { return pin == A0 ? 512 : 520; }
inline int digitalPinToInterrupt(byte pin) { return pin; }
inline void attachInterrupt(int, void (*)(), int) {}
inline void noInterrupts() {}
inline void interrupts() {}
inline void delayMicroseconds(int) {}
struct FakeSerial {
  std::ostringstream output;
  void begin(int) {}
  int available() { return 0; }
  char read() { return 0; }
  template<class T> void print(T value) { output << value; }
  void print(unsigned char value) { output << unsigned(value); }
  void print(float value, byte decimals) { output << std::fixed << std::setprecision(decimals) << value; }
  void println() { output << "\r\n"; }
  template<class T> void println(T value) { print(value); println(); }
  void clear() { output.str(""); output.clear(); }
};
inline FakeSerial Serial;
