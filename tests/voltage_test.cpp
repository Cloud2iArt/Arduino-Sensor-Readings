#include <cassert>
#include <iostream>
#include <algorithm>
#include "../SensorMeasurements.ino"
int main() {
  Wire.present[0x41] = false;
  Wire.present[0x40] = true;
  Wire.raw[0x41] = 9876; // 12.345 V, motor 1
  Wire.raw[0x40] = 9589; // 11.98625 V, motor 2
  setupVoltageSensors();
  assert(Serial.output.str() == "ERROR,INA226_NOT_FOUND,1\r\n");
  sampleTelemetry(); finishTelemetryWindow();
  assert(isnan(Voltage[0]));
  assert(fabs(Voltage[1] - 11.98625F) < 0.00001F);
  Serial.clear(); printTelemetry(); printRawTelemetry();
  auto text = Serial.output.str();
  assert(text.find(",nan,11.986,") != std::string::npos);
  assert(text.find("RAW,0,UNAVAILABLE,9589,512,520\r\n") != std::string::npos);
  Wire.present[0x41] = true;
  fakeNow=999; sampleTelemetry();
  assert(Wire.configs[0x41] == 1); // no tight retry loop
  fakeNow=1000; sampleTelemetry(); // configure recovered sensor
  fakeNow=1005; sampleTelemetry(); finishTelemetryWindow();
  assert(fabs(Voltage[0] - 12.345F) < 0.00001F);
  assert(fabs(Voltage[1] - 11.98625F) < 0.00001F);
  Serial.clear(); printTelemetry(); printRawTelemetry();
  text=Serial.output.str();
  assert(text.find(",12.345,11.986,") != std::string::npos);
  assert(text.find("RAW,1005,9876,9589,512,520\r\n") != std::string::npos);
  // Failed runtime reads must not reuse last window's voltage or RAW value.
  Wire.shortRead[0x41]=true;
  Wire.raw[0x40]=0;
  sampleTelemetry(); finishTelemetryWindow();
  assert(isnan(Voltage[0]) && Voltage[1] == 0);
  Serial.clear(); printTelemetry(); printRawTelemetry();
  assert(Serial.output.str().find("UNAVAILABLE,0,512,520") != std::string::npos);
  Wire.shortRead[0x41]=false;
  fakeNow=2005; sampleTelemetry();
  fakeNow=2010; sampleTelemetry(); finishTelemetryWindow();
  assert(fabs(Voltage[0]-12.345F) < 0.00001F && Voltage[1] == 0);
  Serial.clear(); lastTelemetryMs=fakeNow; loop();
  assert(Serial.output.str().empty()); // no per-loop debug flooding
  fakeNow += 500; loop();
  text=Serial.output.str();
  assert(text.find("DATA,") == 0 && text.find("RAW,") != std::string::npos);
  auto newline=text.find("\r\n");
  assert(newline != std::string::npos);
  assert(std::count(text.begin(), text.begin()+newline, ',') == 14);
  assert(text.substr(newline-12, 12) == ",nan,nan,nan");
  assert(text.find("RAW,") == newline+2);
  assert(text.find("Current Value") == std::string::npos);
  std::cout << "PASS: independent I2C channels, scaling, zero, startup/runtime recovery, per-window availability, DATA/RAW framing and quiet loop\n";
}
