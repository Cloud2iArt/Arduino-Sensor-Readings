#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "ForceReader.h"

// ----------------------------- Pin assignment ------------------------------
const byte RPM_PINS[2] = {2, 3};              // TCRT5000 digital outputs
const byte HX711_DATA_PINS[4] = {4, 5, 6, 8}; // One DOUT per amplifier
const byte HX711_CLOCK_PIN = 7;                // Shared PD_SCK
const byte CURRENT_PINS[2] = {A0, A1};         // ACS724 analogue outputs
// INA226 SDA and SCL use the board's hardware I2C pins (A4 and A5 on an Uno).
// ---------------------------- Load-cell system -----------------------------
// Replace these four values after calibrating each load cell. They are signed
// raw HX711 counts per kilogram; leaving a value at 1 does NOT give calibrated N.
const float LOAD_CELL_COUNTS_PER_KG[4] = {1.0F, 1.0F, 1.0F, 1.0F};
const byte LOAD_CELL_SAMPLES = 1;
ForceReader loadCells(HX711_DATA_PINS, 4, HX711_CLOCK_PIN);
float TORQUE_LOAD_CELL[2] = {NAN, NAN};
float THRUST_LOAD_CELL[2] = {NAN, NAN};
bool loadCellsTared = false;
void setupLoadCells() {
  loadCells.begin(LOAD_CELL_COUNTS_PER_KG);
  Serial.println(F("INFO,Remove all loads; taring four load cells"));
  loadCellsTared = loadCells.tare(10, 250);
  if (!loadCellsTared) Serial.println(F("ERROR,HX711_TARE"));
}
bool readLoadCellForces() {
  return loadCellsTared && loadCells.readForcePairsNewtons(
    TORQUE_LOAD_CELL, THRUST_LOAD_CELL, LOAD_CELL_SAMPLES, 0);
}

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
// Fixed motor order confirmed by the hardware owner; never infer it from scan order.
const byte INA226_ADDRESSES[2] = {0x55, 0x40}; // Motor 1, motor 2; user-confirmed hardware addresses.
const unsigned long INA226_RETRY_INTERVAL_MS = 1000;
const byte INA226_CONFIG_REGISTER = 0x00;
const byte INA226_BUS_VOLTAGE_REGISTER = 0x02;
const uint16_t INA226_CONTINUOUS_CONFIG = 0x4127;
const float INA226_BUS_VOLTAGE_LSB = 0.00125F;
bool ina226Ready[2] = {false, false};
unsigned long ina226LastAttemptMs[2] = {0, 0};
uint16_t voltageRaw[2] = {0, 0};
bool voltageRawAvailable[2] = {false, false};
int currentRaw[2] = {0, 0};
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
  const byte highByte = Wire.read();
  const byte lowByte = Wire.read();
  value = (static_cast<uint16_t>(highByte) << 8) | lowByte;
  return true;
}

void setupVoltageSensors() {
  Wire.begin(); // A4=SDA and A5=SCL on Arduino Uno/Nano.
  Wire.setClock(100000UL);
  Wire.setWireTimeout(25000UL, true); // Bound stuck I2C transactions (AVR core 1.8.6).
  for (byte sensor = 0; sensor < 2; ++sensor) {
    ina226LastAttemptMs[sensor] = millis();
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
  volts = NAN;
  if (!ina226Ready[sensor]) {
    const unsigned long now = millis();
    if (now - ina226LastAttemptMs[sensor] < INA226_RETRY_INTERVAL_MS) return false;
    ina226LastAttemptMs[sensor] = now;
    ina226Ready[sensor] = writeIna226Register(
        INA226_ADDRESSES[sensor], INA226_CONFIG_REGISTER, INA226_CONTINUOUS_CONFIG);
    // Allow the first conversion to complete before using the register.
    return false;
  }
  uint16_t raw;
  if (!readIna226Register(INA226_ADDRESSES[sensor],
                         INA226_BUS_VOLTAGE_REGISTER, raw)) {
    ina226Ready[sensor] = false;
    ina226LastAttemptMs[sensor] = millis();
    return false;
  }
  voltageRaw[sensor] = raw;
  voltageRawAvailable[sensor] = true;
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
    currentRaw[sensor] = analogRead(CURRENT_PINS[sensor]);
    total += currentRaw[sensor];
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
    if (isfinite(value)) {
      total += value;
      ++count;
    }
  }
  float value() const { return count ? total / count : NAN; }
  void reset() { total = 0.0F; count = 0; }
};
SampleAverage torqueSamples[2], thrustSamples[2];
SampleAverage voltageSamples[2], currentSamples[2];
void sampleTelemetry() {
  // A zero timeout polls HX711 readiness without waiting for a conversion.
  const bool forceValid = readLoadCellForces();
  for (byte sensor = 0; sensor < 2; ++sensor) {
    if (forceValid) {
      torqueSamples[sensor].add(TORQUE_LOAD_CELL[sensor]);
      thrustSamples[sensor].add(THRUST_LOAD_CELL[sensor]);
    }
    float volts;
    if (readBusVoltage(sensor, volts)) voltageSamples[sensor].add(volts);
    currentSamples[sensor].add(readCurrentAmps(sensor));
  }
}

void finishTelemetryWindow() {
  for (byte sensor = 0; sensor < 2; ++sensor) {
    TORQUE_LOAD_CELL[sensor] = torqueSamples[sensor].value();
    THRUST_LOAD_CELL[sensor] = thrustSamples[sensor].value();
    Voltage[sensor] = voltageSamples[sensor].value();
    Current[sensor] = currentSamples[sensor].value();
    torqueSamples[sensor].reset();
    thrustSamples[sensor].reset();
    voltageSamples[sensor].reset();
    currentSamples[sensor].reset();
  }
}

const float TORQUE_ARM_METERS[2] = {0.031569F, 0.031569F}; // Replace with measured lever arms.
void printIdentity() { Serial.println(F("DEVICE,SENSORS,2")); }
void processCommand(char *command) {
  if (strcmp(command, "ID?") == 0) printIdentity();
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
  for (byte i = 0; i < 2; ++i) printValue(TORQUE_LOAD_CELL[i] * TORQUE_ARM_METERS[i], 3);
  for (byte i = 0; i < 2; ++i) printValue(THRUST_LOAD_CELL[i], 3);
  for (byte i = 0; i < 2; ++i) printValue(rpm[i], 1);
  for (byte i = 0; i < 2; ++i) printValue(Voltage[i], 3);
  for (byte i = 0; i < 2; ++i) printValue(Current[i], 3);
  Serial.println(F(",nan,nan,nan")); // Throttles and E-stop belong to motor Uno.
}
void printRawTelemetry() {
  Serial.print(F("RAW,")); Serial.print(millis());
  for (byte sensor = 0; sensor < 2; ++sensor) {
    Serial.print(',');
    if (voltageRawAvailable[sensor]) Serial.print(voltageRaw[sensor]);
    else Serial.print(F("UNAVAILABLE"));
    voltageRawAvailable[sensor] = false;
  }
  for (byte sensor = 0; sensor < 2; ++sensor) {
    Serial.print(','); Serial.print(currentRaw[sensor]);
  }
  Serial.println();
}
void setup() {
  Serial.begin(115200);
  printIdentity();
  setupVoltageSensors();
  setupCurrentSensors();
  setupLoadCells();
  setupRpmSensors();
  lastTelemetryMs = rpmSampleStartedMs;
  Serial.println(F("READY,SENSORS"));
}
void loop() {
  serviceSerialCommands();
  sampleTelemetry();
  const unsigned long now = millis();
  if (now - lastTelemetryMs >= TELEMETRY_INTERVAL_MS) {
    lastTelemetryMs = now;
    updateRpm(now);
    finishTelemetryWindow();
    printTelemetry();
    printRawTelemetry();
  }
}

