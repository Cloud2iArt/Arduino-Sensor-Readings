#ifndef FORCE_READER_H
#define FORCE_READER_H
#pragma once
#include <Arduino.h>

// Small, dependency-free HX711 interface intended for use inside a larger
// Arduino application. It does not print, allocate memory, or define setup()/loop().
// Reads several HX711s which share one clock. All DOUT pins are sampled on the
// same clock edge; reading the amplifiers one-by-one would discard conversions
// from the other devices every time the shared clock was pulsed.
class ForceReader {
 public:
  static const byte MAX_CHANNELS = 4;
  enum Channel : byte {
    M1_THRUST = 0,
    M1_TORQUE = 1,
    M2_THRUST = 2,
    M2_TORQUE = 3
  };
  ForceReader(const byte *dataPins, byte channelCount, byte clockPin);
  void begin();
  bool tare(byte sampleCount = 32, unsigned long timeoutMs = 100);
  void clearTare();
  // Output uses the per-load-cell linear calibration equations in
  // ForceReader.cpp, then subtracts the current tare zero:
  // F = (a * F_raw + b) * 9.81 - tareZero. Thrust then subtracts the
  // fitted contribution from the same motor's tared torque load-cell force.
  // Tare all channels together with no applied load before measuring.
  // Each output array must contain channelCount() elements. Output is only
  // updated after a complete, valid sample from every HX711.
  bool readForceNewtons(float *forceNewtons, byte sampleCount = 1,
                        unsigned long timeoutMs = 25);
  // Read four channels into two two-element arrays. Raw channels are mapped as:
  // 0=M1 thrust, 1=M1 torque, 2=M2 thrust, 3=M2 torque. The first array is
  // populated as M1/M2 torque; the second array as M1/M2 thrust.
  bool readForcePairsNewtons(float *torqueNewtons,
                             float *thrustNewtons,
                             byte sampleCount = 1,
                             unsigned long timeoutMs = 25);

  bool readRawAverage(long *rawCounts, byte sampleCount = 1,
                      unsigned long timeoutMs = 25);
  byte channelCount() const { return channelCount_; }

 private:
  bool waitUntilReady(unsigned long timeoutMs) const;
  void readRawSimultaneously(long *rawCounts);
  byte dataPins_[MAX_CHANNELS];
  float tareOffsets_[MAX_CHANNELS];
  byte channelCount_;
  byte clockPin_;
};
#endif
