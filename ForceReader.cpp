#include "ForceReader.h"

namespace {
struct CalibrationEquation {
  float slope;
  float intercept;
};

const float STANDARD_GRAVITY = 9.81F;

// F = (a * F_raw + b) * g
const CalibrationEquation M1_THRUST_CALIBRATION = {-4.032515e-7F, -15.7562F};
const CalibrationEquation M1_TORQUE_CALIBRATION = {-1.665536e-8F, -4.4866F};
const CalibrationEquation M2_THRUST_CALIBRATION = {-5.944979e-7F, 2.1752F};
const CalibrationEquation M2_TORQUE_CALIBRATION = {-1.345070e-8F, -2.3490F};

float applyCalibration(long rawReading, const CalibrationEquation &calibration) {
  return (calibration.slope * rawReading + calibration.intercept) *
         STANDARD_GRAVITY;
}
}

ForceReader::ForceReader(const byte *dataPins, byte channelCount, byte clockPin)
    : channelCount_(channelCount > MAX_CHANNELS ? MAX_CHANNELS : channelCount),
      clockPin_(clockPin) {
  for (byte channel = 0; channel < MAX_CHANNELS; ++channel) {
    dataPins_[channel] = channel < channelCount_ ? dataPins[channel] : 0;
    tareOffsets_[channel] = 0.0F;
  }
}

void ForceReader::begin() {
  pinMode(clockPin_, OUTPUT);
  digitalWrite(clockPin_, LOW);  // HIGH for >60 us powers down every HX711.
  for (byte channel = 0; channel < channelCount_; ++channel) {
    pinMode(dataPins_[channel], INPUT);
  }
}

bool ForceReader::tare(byte sampleCount, unsigned long timeoutMs) {
  long readings[MAX_CHANNELS];
  if (channelCount_ != MAX_CHANNELS ||
      !readRawAverage(readings, sampleCount, timeoutMs)) {
    return false;
  }

  tareOffsets_[0] = applyCalibration(readings[0], M1_THRUST_CALIBRATION);
  tareOffsets_[1] = applyCalibration(readings[1], M1_TORQUE_CALIBRATION);
  tareOffsets_[2] = applyCalibration(readings[2], M2_THRUST_CALIBRATION);
  tareOffsets_[3] = applyCalibration(readings[3], M2_TORQUE_CALIBRATION);
  return true;
}

void ForceReader::clearTare() {
  for (byte channel = 0; channel < MAX_CHANNELS; ++channel) {
    tareOffsets_[channel] = 0.0F;
  }
}

bool ForceReader::readForceNewtons(float *forceNewtons, byte sampleCount,
                                   unsigned long timeoutMs) {
  long readings[MAX_CHANNELS];
  if (!readRawAverage(readings, sampleCount, timeoutMs)) return false;
  if (channelCount_ == MAX_CHANNELS) {
    forceNewtons[0] = applyCalibration(readings[0], M1_THRUST_CALIBRATION) -
                      tareOffsets_[0];
    forceNewtons[1] = applyCalibration(readings[1], M1_TORQUE_CALIBRATION) -
                      tareOffsets_[1];
    forceNewtons[2] = applyCalibration(readings[2], M2_THRUST_CALIBRATION) -
                      tareOffsets_[2];
    forceNewtons[3] = applyCalibration(readings[3], M2_TORQUE_CALIBRATION) -
                      tareOffsets_[3];
    return true;
  }
  return false;
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

  firstPairNewtons[0] = applyCalibration(readings[1], M1_TORQUE_CALIBRATION) -
                        tareOffsets_[1];
  firstPairNewtons[1] = applyCalibration(readings[3], M2_TORQUE_CALIBRATION) -
                        tareOffsets_[3];
  secondPairNewtons[0] = applyCalibration(readings[0], M1_THRUST_CALIBRATION) -
                         tareOffsets_[0];
  secondPairNewtons[1] = applyCalibration(readings[2], M2_THRUST_CALIBRATION) -
                         tareOffsets_[2];
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
