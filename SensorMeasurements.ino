#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "ForceReader.h"

// ----------------------------- Pin assignment ------------------------------
const byte RPM_PINS[2] = {2, 3};              // TCRT5000 digital outputs
// HX711 DOUT: M1 thrust=4, M1 torque=5, M2 thrust=6, M2 torque=8.
// HX711 shared clock=7; raw transport wiring is in ForceReader.cpp.
const byte CURRENT_PINS[2] = {A0, A1};         // ACS724 analogue outputs
// INA226 SDA and SCL use the board's hardware I2C pins (A4 and A5 on an Uno).
// ---------------------------- Load-cell system -----------------------------
// Each load has its own raw count, tare zero and force in newtons (N).
// Calibration input = raw ADC count * 9.81, as specified for these equations.
// Force (N) = slope * (raw ADC count * 9.81) + intercept (N).
// Scale the input before calibration; do not multiply the intercept or output.
// Signed slopes preserve calibration direction.
// Tare with the rig unloaded: subtract the calibrated unloaded force from
// subsequent readings. The intercept cancels, but is shown in both equations.
const byte LOAD_CELL_TARE_SAMPLES = 32; // Average noise in the unloaded zero.
const unsigned long LOAD_CELL_STARTUP_SETTLE_MS = 1000;
const float M1_TORQUE_ARM_METERS = 0.031569F;
const float M2_TORQUE_ARM_METERS = 0.031569F;
ForceReader loadCells; // Only handles synchronized raw ADC acquisition.
bool tareRequested = false; // Serial TARE is performed explicitly in loop().
long m1ThrustRaw = 0;
float m1ThrustTareNewtons = 0.0F;
float m1ThrustNewtons = NAN; // No force sample yet.
long m1TorqueRaw = 0;
float m1TorqueTareNewtons = 0.0F;
float m1TorqueNewtons = NAN; // No force sample yet.
long m2ThrustRaw = 0;
float m2ThrustTareNewtons = 0.0F;
float m2ThrustNewtons = NAN; // No force sample yet.
long m2TorqueRaw = 0;
float m2TorqueTareNewtons = 0.0F;
float m2TorqueNewtons = NAN; // No force sample yet.

// ------------------------------ RPM sensors --------------------------------
const unsigned long TELEMETRY_INTERVAL_MS = 500;
const unsigned long RPM_MIN_EDGE_SPACING_US = 1000; // Reject module chatter.
const unsigned int RPM_PULSES_PER_REVOLUTION[2] = {1, 1};
volatile unsigned long rpmPulseCount[2] = {0, 0};
volatile unsigned long rpmLastEdgeUs[2] = {0, 0};
float rpm[2] = {0.0F, 0.0F};
unsigned long rpmSampleStartedMs = 0;
void countRpmEdge(byte sensor) {
  const unsigned long now = micros();
  if (now - rpmLastEdgeUs[sensor] >= RPM_MIN_EDGE_SPACING_US) {
    ++rpmPulseCount[sensor];
    rpmLastEdgeUs[sensor] = now;
  }
}
void rpm1Interrupt() { countRpmEdge(0); }
void rpm2Interrupt() { countRpmEdge(1); }
void setupRpmSensors() {
  pinMode(RPM_PINS[0], INPUT_PULLUP);
  pinMode(RPM_PINS[1], INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RPM_PINS[0]), rpm1Interrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(RPM_PINS[1]), rpm2Interrupt, FALLING);
  rpmSampleStartedMs = millis();
}
void updateRpm(unsigned long now) {
  const unsigned long elapsed = now - rpmSampleStartedMs;
  if (elapsed == 0) return;
  unsigned long pulses[2];
  noInterrupts();
  pulses[0] = rpmPulseCount[0];
  pulses[1] = rpmPulseCount[1];
  rpmPulseCount[0] = rpmPulseCount[1] = 0;
  interrupts();
  for (byte sensor = 0; sensor < 2; ++sensor) {
    rpm[sensor] = (pulses[sensor] * 60000.0F) /
                  (elapsed * RPM_PULSES_PER_REVOLUTION[sensor]);
  }
  rpmSampleStartedMs = now;
}

// -------------------------- INA226 voltage inputs --------------------------
const byte INA226_ADDRESSES[2] = {0x55, 0x40}; // Set A0/A1 straps differently.
const byte INA226_CONFIG_REGISTER = 0x00;
const byte INA226_BUS_VOLTAGE_REGISTER = 0x02;
const uint16_t INA226_CONTINUOUS_CONFIG = 0x4127;
const float INA226_BUS_VOLTAGE_LSB = 0.00125F;
bool ina226Ready[2] = {false, false};
float Voltage[2] = {NAN, NAN};
bool writeIna226Register(byte address, byte reg, uint16_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(static_cast<byte>(value >> 8));
  Wire.write(static_cast<byte>(value));
  return Wire.endTransmission() == 0;
}
  bool readIna226Register(byte address, byte reg, uint16_t &value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
 if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<byte>(2)) != 2) return false;
  value = (static_cast<uint16_t>(Wire.read()) << 8) | Wire.read();
  return true;
}

void setupVoltageSensors() {
  Wire.begin(); // A4=SDA and A5=SCL on Arduino Uno/Nano.
  Wire.setClock(100000UL);
  Wire.setWireTimeout(25000UL, true); // Bound stuck I2C transactions (AVR core 1.8.6).
  for (byte sensor = 0; sensor < 2; ++sensor) {
    ina226Ready[sensor] = writeIna226Register(
        INA226_ADDRESSES[sensor], INA226_CONFIG_REGISTER,
        INA226_CONTINUOUS_CONFIG);
    if (!ina226Ready[sensor]) {
      Serial.print(F("ERROR,INA226_NOT_FOUND,"));
      Serial.println(sensor + 1);
    }
  }
}

bool readBusVoltage(byte sensor, float &volts) {
  uint16_t raw;
  if (!ina226Ready[sensor] ||
      !readIna226Register(INA226_ADDRESSES[sensor],
                          INA226_BUS_VOLTAGE_REGISTER, raw)) return false;
  volts = raw * INA226_BUS_VOLTAGE_LSB;
  return true;
}

// -------------------------- ACS724 current inputs --------------------------
const float ADC_REFERENCE_VOLTS = 5.0F;
const float ACS724_ZERO_VOLTS[2] = {2.5F, 2.5F};
// 0.400 V/A suits ACS724LLCTR-05AB. Change for the exact fitted variants.
const float ACS724_VOLTS_PER_AMP[2] = {0.400F, 0.400F};
const byte CURRENT_SAMPLE_COUNT = 16;
float Current[2] = {NAN, NAN};
void setupCurrentSensors() {
  pinMode(CURRENT_PINS[0], INPUT);
  pinMode(CURRENT_PINS[1], INPUT);
}
float readCurrentAmps(byte sensor) {
  unsigned long total = 0;
  for (byte sample = 0; sample < CURRENT_SAMPLE_COUNT; ++sample) {
    total += analogRead(CURRENT_PINS[sensor]);
  }
  const float volts = (total / static_cast<float>(CURRENT_SAMPLE_COUNT)) *
                      ADC_REFERENCE_VOLTS / 1023.0F;
  return (volts - ACS724_ZERO_VOLTS[sensor]) /
         ACS724_VOLTS_PER_AMP[sensor];
}


unsigned long lastTelemetryMs = 0;
// Separate counts exclude failed reads instead of reusing stale values.
struct SampleAverage {
  float total = 0.0F;
  unsigned long count = 0;
  void add(float value) {
    if (!isnan(value)) {
      total += value;
      ++count;
    }
  }
  float value() const { return count ? total / count : NAN; }
  void reset() { total = 0.0F; count = 0; }
};
SampleAverage m1ThrustSamples; // Independent valid-sample average.
SampleAverage m1TorqueSamples; // Independent valid-sample average.
SampleAverage m2ThrustSamples; // Independent valid-sample average.
SampleAverage m2TorqueSamples; // Independent valid-sample average.
SampleAverage voltageSamples[2], currentSamples[2];
void sampleTelemetry() {
  for (byte sensor = 0; sensor < 2; ++sensor) {
    float volts;
    if (readBusVoltage(sensor, volts)) voltageSamples[sensor].add(volts);
    currentSamples[sensor].add(readCurrentAmps(sensor));
  }
}

void finishTelemetryWindow() {
  // Publish each independent mean; an empty window reports nan.
  m1ThrustNewtons = m1ThrustSamples.value();
  m1ThrustSamples.reset();
  m1TorqueNewtons = m1TorqueSamples.value();
  m1TorqueSamples.reset();
  m2ThrustNewtons = m2ThrustSamples.value();
  m2ThrustSamples.reset();
  m2TorqueNewtons = m2TorqueSamples.value();
  m2TorqueSamples.reset();

  for (byte sensor = 0; sensor < 2; ++sensor) {
    Voltage[sensor] = voltageSamples[sensor].value();
    Current[sensor] = currentSamples[sensor].value();
    voltageSamples[sensor].reset();
    currentSamples[sensor].reset();
  }
}

void resetTelemetryAverages() {
  m1ThrustSamples.reset();
  m1TorqueSamples.reset();
  m2ThrustSamples.reset();
  m2TorqueSamples.reset();

  for (byte sensor = 0; sensor < 2; ++sensor) {
    voltageSamples[sensor].reset();
    currentSamples[sensor].reset();
  }
}

void printRawLoadCellDiagnostics() {
  long m1ThrustDiagnostic;
  long m1TorqueDiagnostic;
  long m2ThrustDiagnostic;
  long m2TorqueDiagnostic;
  if (!loadCells.readRawAverage(m1ThrustDiagnostic, m1TorqueDiagnostic, m2ThrustDiagnostic, m2TorqueDiagnostic, LOAD_CELL_TARE_SAMPLES, 100)) {
    Serial.println(F("ERROR,RAW_LOAD_CELLS"));
    return;
  }

  Serial.print(F("RAW_LOAD_CELLS,M1_THRUST,M1_TORQUE,M2_THRUST,M2_TORQUE"));
  Serial.print(','); Serial.print(m1ThrustDiagnostic); // Motor 1 thrust ADC counts.
  Serial.print(','); Serial.print(m1TorqueDiagnostic); // Motor 1 torque ADC counts.
  Serial.print(','); Serial.print(m2ThrustDiagnostic); // Motor 2 thrust ADC counts.
  Serial.print(','); Serial.print(m2TorqueDiagnostic); // Motor 2 torque ADC counts.
  Serial.println();
}

void printIdentity() { Serial.println(F("DEVICE,SENSORS,2")); }
void processCommand(char *command) {
  if (strcmp(command, "ID?") == 0) printIdentity();
  else if (strcmp(command, "RAW?") == 0) printRawLoadCellDiagnostics();
  else if (strcmp(command, "TARE") == 0) {
    tareRequested = true; // loop() measures and calculates each tare zero.
  }
  else Serial.println(F("ERROR,BAD_COMMAND"));
}
#include "SerialCommands.h"
void printValue(float value, byte decimals) {
  Serial.print(',');
  if (isfinite(value)) Serial.print(value, decimals);
  else Serial.print(F("nan"));
}
void printTelemetry() {
  Serial.print(F("DATA,")); Serial.print(millis());
  // UI order: torque1, torque2, thrust1, thrust2.
  // Torque (N m) = tangential load (N) * perpendicular arm length (m).
  // Six decimals retain small torque responses that rounded to 0.000 N m.
  printValue(m1TorqueNewtons * M1_TORQUE_ARM_METERS, 6);
  printValue(m2TorqueNewtons * M2_TORQUE_ARM_METERS, 6);
  printValue(m1ThrustNewtons, 3); // Axial thrust already in newtons.
  printValue(m2ThrustNewtons, 3);
  for (byte i = 0; i < 2; ++i) printValue(rpm[i], 1);
  for (byte i = 0; i < 2; ++i) printValue(Voltage[i], 3);
  for (byte i = 0; i < 2; ++i) printValue(Current[i], 3);
  Serial.println(F(",nan,nan,nan")); // Throttles and E-stop belong to motor Uno.
}
void setup() {
  Serial.begin(115200);
  printIdentity();
  setupVoltageSensors();
  setupCurrentSensors();
  loadCells.begin();
  delay(LOAD_CELL_STARTUP_SETTLE_MS); // Allow amplifiers to settle before zeroing.
  // Keep each old tare if acquisition fails; replace zeros only on success.
  if (loadCells.readRawAverage(m1ThrustRaw, m1TorqueRaw, m2ThrustRaw, m2TorqueRaw,
                               LOAD_CELL_TARE_SAMPLES, 100)) {
    // Motor 1 thrust: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
    m1ThrustTareNewtons = (-4.032515e-7F * (m1ThrustRaw * 9.81F) + (-15.7562F));
    // Motor 1 torque: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
    m1TorqueTareNewtons = (-1.665536e-8F * (m1TorqueRaw * 9.81F) + (-4.4866F));
    // Motor 2 thrust: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
    m2ThrustTareNewtons = (-5.944979e-7F * (m2ThrustRaw * 9.81F) + (2.1752F));
    // Motor 2 torque: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
    m2TorqueTareNewtons = (-1.345070e-8F * (m2TorqueRaw * 9.81F) + (-2.3490F));
    Serial.println(F("READY,LOAD_CELLS_TARED"));
  } else {
    Serial.println(F("ERROR,LOAD_CELL_TARE_FAILED"));
  }
  setupRpmSensors();
  lastTelemetryMs = rpmSampleStartedMs;
  Serial.println(F("READY,SENSORS"));
}
void loop() {
  serviceSerialCommands();
  if (tareRequested) {
    tareRequested = false;
    // Keep each old tare if acquisition fails; replace zeros only on success.
    if (loadCells.readRawAverage(m1ThrustRaw, m1TorqueRaw, m2ThrustRaw, m2TorqueRaw,
                                 LOAD_CELL_TARE_SAMPLES, 100)) {
      // Motor 1 thrust: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
      m1ThrustTareNewtons = (-4.032515e-7F * (m1ThrustRaw * 9.81F) + (-15.7562F));
      // Motor 1 torque: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
      m1TorqueTareNewtons = (-1.665536e-8F * (m1TorqueRaw * 9.81F) + (-4.4866F));
      // Motor 2 thrust: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
      m2ThrustTareNewtons = (-5.944979e-7F * (m2ThrustRaw * 9.81F) + (2.1752F));
      // Motor 2 torque: tare (N) = slope * (mean unloaded ADC * 9.81) + intercept (N).
      m2TorqueTareNewtons = (-1.345070e-8F * (m2TorqueRaw * 9.81F) + (-2.3490F));
      resetTelemetryAverages(); // Never mix samples from before and after tare.
      Serial.println(F("OK,TARE"));
    } else {
      Serial.println(F("ERROR,TARE_FAILED"));
    }
  }
  // Poll the shared-clock acquisition; no ready conversion means no new sample.
  if (loadCells.readRawAverage(m1ThrustRaw, m1TorqueRaw, m2ThrustRaw, m2TorqueRaw, 1, 0)) {
    // Motor 1 thrust: scale raw ADC by 9.81, calibrate to N, then subtract its own tare (N).
    m1ThrustNewtons = (-4.032515e-7F * (m1ThrustRaw * 9.81F) + (-15.7562F)) - m1ThrustTareNewtons;
    m1ThrustSamples.add(m1ThrustNewtons);
    // Motor 1 torque: scale raw ADC by 9.81, calibrate to N, then subtract its own tare (N).
    m1TorqueNewtons = (-1.665536e-8F * (m1TorqueRaw * 9.81F) + (-4.4866F)) - m1TorqueTareNewtons;
    m1TorqueSamples.add(m1TorqueNewtons);
    // Motor 2 thrust: scale raw ADC by 9.81, calibrate to N, then subtract its own tare (N).
    m2ThrustNewtons = (-5.944979e-7F * (m2ThrustRaw * 9.81F) + (2.1752F)) - m2ThrustTareNewtons;
    m2ThrustSamples.add(m2ThrustNewtons);
    // Motor 2 torque: scale raw ADC by 9.81, calibrate to N, then subtract its own tare (N).
    m2TorqueNewtons = (-1.345070e-8F * (m2TorqueRaw * 9.81F) + (-2.3490F)) - m2TorqueTareNewtons;
    m2TorqueSamples.add(m2TorqueNewtons);
  }
  sampleTelemetry();
  const unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    updateRpm(now);
    finishTelemetryWindow();
    printTelemetry();

  }
}
