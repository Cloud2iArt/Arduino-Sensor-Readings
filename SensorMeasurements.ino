#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <string.h>
#include "ForceReader.h"

// ----------------------------- Pin assignment ------------------------------
const byte RPM_PINS[2] = {2, 3};              // TCRT5000 digital outputs
// ForceReader channel order: M1 thrust, M1 torque, M2 thrust, M2 torque.
const byte HX711_DATA_PINS[4] = {6, 4, 8, 5}; // One DOUT per amplifier
const byte HX711_CLOCK_PIN = 7;                // Shared PD_SCK
const byte CURRENT_PINS[2] = {A0, A1};         // ACS724 analogue outputs
// INA226 SDA and SCL use the board's hardware I2C pins (A4 and A5 on an Uno).
// ---------------------------- Load-cell system -----------------------------
// Load-cell output uses per-cell calibration equations in ForceReader.cpp.
const byte LOAD_CELL_SAMPLES = 1;
const byte LOAD_CELL_TARE_SAMPLES = 32;
const unsigned long LOAD_CELL_STARTUP_SETTLE_MS = 1000;
const float TORQUE_ARM_METERS[2] = {1.0F, 1.0F};
//const float TORQUE_ARM_METERS[2] = {0.031569F, 0.031569F};
ForceReader loadCells(HX711_DATA_PINS, 4, HX711_CLOCK_PIN);
float TORQUE_LOAD_CELL[2] = {NAN, NAN};
float THRUST_LOAD_CELL[2] = {NAN, NAN};
void setupLoadCells() {
  loadCells.begin();
  delay(LOAD_CELL_STARTUP_SETTLE_MS);
  if (loadCells.tare(LOAD_CELL_TARE_SAMPLES, 100)) {
    Serial.println(F("READY,LOAD_CELLS_TARED"));
  } else {
    Serial.println(F("ERROR,LOAD_CELL_TARE_FAILED"));
  }
}
bool readLoadCellForces(float *torqueNewtons, float *thrustNewtons) {
  return loadCells.readForcePairsNewtons(
    torqueNewtons, thrustNewtons, LOAD_CELL_SAMPLES, 0);
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
SampleAverage torqueSamples[2], thrustSamples[2];
SampleAverage voltageSamples[2], currentSamples[2];
void sampleTelemetry() {
  // A zero timeout polls HX711 readiness without waiting for a conversion.
  float torqueNewtons[2], thrustNewtons[2];
  const bool forceValid = readLoadCellForces(torqueNewtons, thrustNewtons);
  for (byte sensor = 0; sensor < 2; ++sensor) {
    if (forceValid) {
      torqueSamples[sensor].add(torqueNewtons[sensor]);
      thrustSamples[sensor].add(thrustNewtons[sensor]);
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

void resetTelemetryAverages() {
  for (byte sensor = 0; sensor < 2; ++sensor) {
    torqueSamples[sensor].reset();
    thrustSamples[sensor].reset();
    voltageSamples[sensor].reset();
    currentSamples[sensor].reset();
  }
}

void printRawLoadCellDiagnostics() {
  long rawCounts[4];
  if (!loadCells.readRawAverage(rawCounts, LOAD_CELL_TARE_SAMPLES, 100)) {
    Serial.println(F("ERROR,RAW_LOAD_CELLS"));
    return;
  }

  Serial.print(F("RAW_LOAD_CELLS,M1_THRUST,M1_TORQUE,M2_THRUST,M2_TORQUE"));
  for (byte channel = 0; channel < 4; ++channel) {
    Serial.print(',');
    Serial.print(rawCounts[channel]);
  }
  Serial.println();
}

void printIdentity() { Serial.println(F("DEVICE,SENSORS,2")); }
void processCommand(char *command) {
  if (strcmp(command, "ID?") == 0) printIdentity();
  else if (strcmp(command, "RAW?") == 0) printRawLoadCellDiagnostics();
  else if (strcmp(command, "TARE") == 0) {
    if (loadCells.tare(LOAD_CELL_TARE_SAMPLES, 100)) {
      resetTelemetryAverages();
      Serial.println(F("OK,TARE"));
    } else {
      Serial.println(F("ERROR,TARE_FAILED"));
    }
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
  for (byte i = 0; i < 2; ++i) {
    printValue(TORQUE_LOAD_CELL[i] * TORQUE_ARM_METERS[i], 5);
  }
  for (byte i = 0; i < 2; ++i) printValue(THRUST_LOAD_CELL[i], 3);
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

  }
}
