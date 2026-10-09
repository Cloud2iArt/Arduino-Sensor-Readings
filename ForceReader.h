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
  ForceReader(const byte *dataPins, byte channelCount, byte clockPin);
  // Calibration is raw counts per kilogram (one value per load cell). A factor
  // may be negative when that cell is wired with the opposite polarity.
  void begin(const float *calibrationFactorsCountsPerKg);
  bool tare(byte sampleCount = 20, unsigned long timeoutMs = 250);
    // Each output array must contain channelCount() elements. Output is only
  // updated after a complete, valid sample from every HX711.
  bool readForceNewtons(float *forceNewtons, byte sampleCount = 1,
                        unsigned long timeoutMs = 25);
  // Read four channels into two two-element arrays. Channels 0/1 populate the
  // first array and channels 2/3 populate the second array.
  bool readForcePairsNewtons(float *firstPairNewtons,
                             float *secondPairNewtons,
                             byte sampleCount = 1,
                             unsigned long timeoutMs = 25);

  bool readRawAverage(long *rawCounts, byte sampleCount = 1,
                      unsigned long timeoutMs = 25);
    byte channelCount() const { return channelCount_; }
  long zeroOffset(byte channel) const;
  void setZeroOffset(byte channel, long zeroOffset);
  void setCalibrationFactor(byte channel, float factorCountsPerKg);

 private:
  bool waitUntilReady(unsigned long timeoutMs) const;
  void readRawSimultaneously(long *rawCounts);
    byte dataPins_[MAX_CHANNELS];
  byte channelCount_;
  byte clockPin_;
   long zeroOffsets_[MAX_CHANNELS];
  float calibrationFactors_[MAX_CHANNELS];
};
#endif 
