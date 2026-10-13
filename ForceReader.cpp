#include "ForceReader.h"

void ForceReader::begin() {
  pinMode(7, OUTPUT); // Shared HX711 PD_SCK.
  digitalWrite(7, LOW); // Holding HIGH for >60 us powers down the amplifiers.
  pinMode(4, INPUT); // Motor 1 thrust DOUT.
  pinMode(5, INPUT); // Motor 1 torque DOUT.
  pinMode(6, INPUT); // Motor 2 thrust DOUT.
  pinMode(8, INPUT); // Motor 2 torque DOUT.
}

bool ForceReader::readRawAverage(long &m1Thrust, long &m1Torque, long &m2Thrust, long &m2Torque,
                                  byte sampleCount, unsigned long timeoutMs) {
  if (sampleCount == 0) return false; // Cannot average zero samples.
  int64_t m1ThrustTotal = 0; // Wide accumulator prevents overflow during tare.
  int64_t m1TorqueTotal = 0; // Wide accumulator prevents overflow during tare.
  int64_t m2ThrustTotal = 0; // Wide accumulator prevents overflow during tare.
  int64_t m2TorqueTotal = 0; // Wide accumulator prevents overflow during tare.
  for (byte sample = 0; sample < sampleCount; ++sample) {
    const unsigned long start = millis();
    // LOW means a conversion is ready. A zero timeout gives a nonblocking poll.
    // Reading one amplifier alone would consume the other three conversions.
    while (digitalRead(4) != LOW || digitalRead(5) != LOW ||
           digitalRead(6) != LOW || digitalRead(8) != LOW) {
      if (millis() - start >= timeoutMs) return false;
    }
    uint32_t m1ThrustBits = 0;
    uint32_t m1TorqueBits = 0;
    uint32_t m2ThrustBits = 0;
    uint32_t m2TorqueBits = 0;
    // Keep clock HIGH pulses short even when RPM interrupts are active.
    noInterrupts();
    for (byte bit = 0; bit < 24; ++bit) {
      digitalWrite(7, HIGH);
      delayMicroseconds(1);
      // Shift each sensor's own serial bit into its own 24-bit reading.
      m1ThrustBits = (m1ThrustBits << 1) | digitalRead(4);
      m1TorqueBits = (m1TorqueBits << 1) | digitalRead(5);
      m2ThrustBits = (m2ThrustBits << 1) | digitalRead(6);
      m2TorqueBits = (m2TorqueBits << 1) | digitalRead(8);
      digitalWrite(7, LOW);
      delayMicroseconds(1);
    }
    // Pulse 25 selects channel A, gain 128, for the next conversion on all four.
    digitalWrite(7, HIGH);
    delayMicroseconds(1);
    digitalWrite(7, LOW);
    interrupts();
    // Sign-extend each two's-complement 24-bit ADC value to signed 32 bits.
    if (m1ThrustBits & 0x00800000UL) m1ThrustBits |= 0xFF000000UL;
    m1ThrustTotal += static_cast<int32_t>(m1ThrustBits);
    if (m1TorqueBits & 0x00800000UL) m1TorqueBits |= 0xFF000000UL;
    m1TorqueTotal += static_cast<int32_t>(m1TorqueBits);
    if (m2ThrustBits & 0x00800000UL) m2ThrustBits |= 0xFF000000UL;
    m2ThrustTotal += static_cast<int32_t>(m2ThrustBits);
    if (m2TorqueBits & 0x00800000UL) m2TorqueBits |= 0xFF000000UL;
    m2TorqueTotal += static_cast<int32_t>(m2TorqueBits);
  }
  // Average each cell independently; publish only after all samples succeed.
  m1Thrust = static_cast<long>(m1ThrustTotal / sampleCount);
  m1Torque = static_cast<long>(m1TorqueTotal / sampleCount);
  m2Thrust = static_cast<long>(m2ThrustTotal / sampleCount);
  m2Torque = static_cast<long>(m2TorqueTotal / sampleCount);
  return true;
}
