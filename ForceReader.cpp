#include "ForceReader.h"

namespace {
const float STANDARD_GRAVITY = 9.80665F;
}

ForceReader::ForceReader(const byte *dataPins, byte channelCount, byte clockPin)
    : channelCount_(channelCount > MAX_CHANNELS ? MAX_CHANNELS : channelCount),
      clockPin_(clockPin) {
  for (byte channel = 0; channel < MAX_CHANNELS; ++channel) {
    dataPins_[channel] = channel < channelCount_ ? dataPins[channel] : 0;
    zeroOffsets_[channel] = 0;
    calibrationFactors_[channel] = 1.0F;
  }
}

 void ForceReader::begin(const float *calibrationFactorsCountsPerKg) {
  pinMode(clockPin_, OUTPUT);
  digitalWrite(clockPin_, LOW);  // HIGH for >60 us powers down every HX711.
  for (byte channel = 0; channel < channelCount_; ++channel) {
    pinMode(dataPins_[channel], INPUT);
    // Assign this directly rather than calling another member during startup.
    // A zero factor is invalid because force conversion divides by this value.
    const float factor = calibrationFactorsCountsPerKg[channel];
    calibrationFactors_[channel] = factor == 0.0F ? 1.0F : factor;
  }

}
 
 bool ForceReader::tare(byte sampleCount, unsigned long timeoutMs) {
  long readings[MAX_CHANNELS];
  if (!readRawAverage(readings, sampleCount, timeoutMs)) return false;
  for (byte channel = 0; channel < channelCount_; ++channel) {
    zeroOffsets_[channel] = readings[channel];
  }
  return true;
 }

bool ForceReader::readForceNewtons(float *forceNewtons, byte sampleCount,
                                   unsigned long timeoutMs) {
  long readings[MAX_CHANNELS];
  if (!readRawAverage(readings, sampleCount, timeoutMs)) return false;
  for (byte channel = 0; channel < channelCount_; ++channel) {
    const float massKg =
        (readings[channel] - zeroOffsets_[channel]) / calibrationFactors_[channel];
    forceNewtons[channel] = massKg * STANDARD_GRAVITY;
  }
  return true;
}

bool ForceReader::readForcePairsNewtons(float *firstPairNewtons,
                                        float *secondPairNewtons,
                                        byte sampleCount,
                                        unsigned long timeoutMs) {
  long readings[MAX_CHANNELS];
  if (channelCount_ != MAX_CHANNELS ||
      !readRawAverage(readings, sampleCount, timeoutMs)) {
    return false;
  }

  for (byte channel = 0; channel < channelCount_; ++channel) {
    const float massKg =
        (readings[channel] - zeroOffsets_[channel]) /
        calibrationFactors_[channel];
    float forceNewtons = massKg * STANDARD_GRAVITY;
    if (channel < 2) {
      // Torque load cells: channels 0 and 1.
      forceNewtons =
          1.50323425e-6F * forceNewtons + 0.29719284F;
    } else {
      // Thrust load cells: channels 2 and 3.
      forceNewtons =
          -6.45519948e-8F * forceNewtons; //+ 3.33527235F//;
    }

    float *outputPair = channel < 2 ? firstPairNewtons
                                    : secondPairNewtons;
    outputPair[channel % 2] = forceNewtons;
  }
  return true;
}
  
bool ForceReader::readRawAverage(long *rawCounts, byte sampleCount,
                                 unsigned long timeoutMs) {
  if (sampleCount == 0 || channelCount_ == 0) return false;
    int64_t totals[MAX_CHANNELS] = {0, 0, 0, 0};
  long readings[MAX_CHANNELS];
  for (byte sample = 0; sample < sampleCount; ++sample) {
     if (!waitUntilReady(timeoutMs)) return false;
    readRawSimultaneously(readings);
    for (byte channel = 0; channel < channelCount_; ++channel) {
      totals[channel] += readings[channel];
    }
  }
  for (byte channel = 0; channel < channelCount_; ++channel) {
    rawCounts[channel] = static_cast<long>(totals[channel] / sampleCount);
  }
  return true;
}

long ForceReader::zeroOffset(byte channel) const {
  return channel < channelCount_ ? zeroOffsets_[channel] : 0;
}


  bool ForceReader::waitUntilReady(unsigned long timeoutMs) const {
  const unsigned long start = millis();
  while (true) {
    bool allReady = true;
    for (byte channel = 0; channel < channelCount_; ++channel) {
      if (digitalRead(dataPins_[channel]) == HIGH) {
        allReady = false;
        break;
      }
    }
   if (allReady) return true;
    if (millis() - start >= timeoutMs) return false;
  }

}
void ForceReader::readRawSimultaneously(long *rawCounts) {
  uint32_t values[MAX_CHANNELS] = {0, 0, 0, 0};

  noInterrupts();
  for (byte bit = 0; bit < 24; ++bit) {
    digitalWrite(clockPin_, HIGH);
    delayMicroseconds(1);
    for (byte channel = 0; channel < channelCount_; ++channel) {
      values[channel] = (values[channel] << 1) | digitalRead(dataPins_[channel]);
    }
    digitalWrite(clockPin_, LOW);
    delayMicroseconds(1);
  }
    // Pulse 25 selects channel A, gain 128, for every amplifier's next sample.
  digitalWrite(clockPin_, HIGH);
  delayMicroseconds(1);
  digitalWrite(clockPin_, LOW);
  interrupts();

  for (byte channel = 0; channel < channelCount_; ++channel) {
    if (values[channel] & 0x00800000UL) values[channel] |= 0xFF000000UL;
    rawCounts[channel] = static_cast<long>(values[channel]);
  }
}