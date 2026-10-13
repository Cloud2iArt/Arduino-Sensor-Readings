#pragma once
#include <Arduino.h>

// Raw HX711 transport only. Calibration and tare are explicit in the sketch.
// All four devices share pin 7, so every clock edge must read all four DOUTs.
class ForceReader {
 public:
  void begin();
  // Return signed ADC counts; leave outputs unchanged if any device times out.
  bool readRawAverage(long &m1Thrust, long &m1Torque, long &m2Thrust, long &m2Torque,
                      byte sampleCount = 1, unsigned long timeoutMs = 25);
};
