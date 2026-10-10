Arduino-to-UI serial data protocol

This document describes the transmission implemented in `SensorMeasurements.ino`
and `SerialCommands.h`. It specifies what the UI must receive and parse; it does
not establish that the existing UI implements this protocol.

## Connection and framing

The sensor Arduino sends plain-text, comma-separated records over its serial
connection (normally USB serial on an Uno).

| Setting | Value |
| --- | --- |
| Baud rate | 115200 |
| Serial configuration | 8 data bits, no parity, 1 stop bit (8N1) |
| Record separator | CRLF (`\r\n`), produced by `Serial.println()` |
| Field separator | Comma (`,`) |
| Decimal separator | Period (`.`) |
| Telemetry interval | Approximately 500 ms after setup completes |

The firmware sends telemetry automatically; polling is not required. Each window
produces one `DATA` record followed by one `RAW` record. Setup includes load-cell
taring, so telemetry does not begin immediately after reset.

Serial reads do not necessarily correspond to complete lines. The UI or its
serial backend must buffer incoming bytes, extract complete newline-terminated
lines, and retain any incomplete suffix for the next read. One read can contain
multiple records. Strip the trailing carriage return before parsing.

## DATA: measurements for the main UI

Each `DATA` record contains exactly **15 fields**, including its record type:

```text
DATA,millis,torque1,torque2,thrust1,thrust2,rpm1,rpm2,volts1,volts2,amps1,amps2,nan,nan,nan
```

All indexes below start at zero after splitting the **entire** line by commas.

| Index | Field | Meaning / units | Printed precision |
| --- | --- | --- | --- |
| 0 | `DATA` | Record type | Text |
| 1 | `millis` | Arduino uptime in milliseconds | Integer |
| 2 | `torque1` | Motor 1 torque, intended N·m | 3 decimals |
| 3 | `torque2` | Motor 2 torque, intended N·m | 3 decimals |
| 4 | `thrust1` | Motor 1 thrust, intended N | 3 decimals |
| 5 | `thrust2` | Motor 2 thrust, intended N | 3 decimals |
| 6 | `rpm1` | Motor 1 revolutions per minute | 1 decimal |
| 7 | `rpm2` | Motor 2 revolutions per minute | 1 decimal |
| **8** | **`volts1`** | **Motor 1 voltage in volts** | **3 decimals** |
| **9** | **`volts2`** | **Motor 2 voltage in volts** | **3 decimals** |
| 10 | `amps1` | Motor 1 current in amperes | 3 decimals |
| 11 | `amps2` | Motor 2 current in amperes | 3 decimals |
| 12 | Throttle 1 placeholder | Always `nan` on this sensor board | Text |
| 13 | Throttle 2 placeholder | Always `nan` on this sensor board | Text |
| 14 | E-stop placeholder | Always `nan` on this sensor board | Text |

If the parser removes the `DATA` token first, subtract one from these indexes:
motor 1 and motor 2 voltage then occupy indexes **7 and 8**.

Voltage, current, torque, and thrust are averaged over the telemetry window using
available finite samples. RPM is calculated from pulse counts and elapsed time.
Calibration constants must match the hardware for physical units to be accurate;
load-cell calibration and torque-arm lengths in particular require configuration.

### Independent values and missing measurements

Motor 1 and motor 2 are separate fields. Never combine them, copy one to both
displays, or infer their order from their relative values.

`nan` means the corresponding measurement is unavailable or non-finite. Parse
valid fields independently: **do not reject the whole record because any field
is `nan`**. The final three fields are deliberately `nan` because throttle and
E-stop belong to a separate motor controller.

Display an unavailable field as `Unavailable` or `—`, rather than zero. A numeric
**0.000 is a valid measurement**. Do not use a truthiness check to test validity.
If forwarding through JSON, represent unavailable values as `null`; bare `NaN`
is not a valid JSON number.

### Example DATA record

This is a synthetic example, not a hardware measurement:

```text
DATA,1500,nan,nan,nan,nan,1200.0,1250.0,12.345,11.987,0.100,0.200,nan,nan,nan
```

The UI should show motor 1 at **12.345 V / 0.100 A** and motor 2 at
**11.987 V / 0.200 A**, even though torque, thrust, and controller fields are
unavailable. Voltage fields are already in volts; do not scale them again.

## RAW: unscaled electrical sensor diagnostics

Each `RAW` record contains six fields:

```text
RAW,millis,voltage_register1,voltage_register2,current_adc1,current_adc2
```

| Index | Meaning |
| --- | --- |
| 0 | `RAW` record type |
| 1 | Arduino uptime in milliseconds |
| 2 | Motor 1: latest successful INA226 bus-voltage register count in the window |
| 3 | Motor 2: latest successful INA226 bus-voltage register count in the window |
| 4 | Motor 1: latest individual ACS724 ADC reading on A0, 0–1023 |
| 5 | Motor 2: latest individual ACS724 ADC reading on A1, 0–1023 |

An unavailable raw voltage is the literal string `UNAVAILABLE`. For the INA226
implementation, one voltage register count represents 0.00125 V. Raw current
counts have not undergone averaging, offset subtraction, or sensitivity scaling.
The firmware uses a nominal 5 V ADC reference, 2.5 V zero-current offset, and
0.400 V/A sensitivity for its converted current readings; these are hardware-
dependent settings, not UI conversion requirements.

Use `DATA` for normal voltage/current displays. Route `RAW` to a separate
optional diagnostics panel; never interpret its counts as volts or amperes.
`DATA` and `RAW` call `millis()` separately, so their timestamps need not match
exactly. Latest raw samples also need not equal the window averages in `DATA`.

## Identity, startup, commands, and errors

Other records share the serial stream and have different lengths:

| Record | Meaning |
| --- | --- |
| `DEVICE,SENSORS,2` | Sensor-board identity, emitted at startup and in reply to `ID?` |
| `INFO,Remove all loads; taring four load cells` | Load-cell initialization message |
| `ERROR,INA226_NOT_FOUND,1` | Motor 1 voltage-sensor initialization failed |
| `ERROR,INA226_NOT_FOUND,2` | Motor 2 voltage-sensor initialization failed |
| `ERROR,HX711_TARE` | Load-cell tare failed |
| `READY,SENSORS` | Setup finished; this does not guarantee that all sensors initialized |
| `ERROR,BAD_COMMAND` | Unrecognized command |
| `ERROR,BAD_LINE` | Command line too long or containing unsupported characters |

The only implemented command is `ID?`, terminated by LF, CR, or CRLF. For example,
send `ID?\n` to identify the board. Commands are case-sensitive, with a maximum
of 31 printable characters before the terminator.

Failed voltage sensors are retried automatically. Later read failures are reflected
in telemetry availability; an error message is not necessarily emitted for each
failure. If multiple Arduinos are connected, preserve source-port identity and
use the sensor board's `DATA` records for these measurements.

## Reference UI parser (JavaScript)

This example accepts decoded text chunks from the sensor serial connection.
Replace `updateSensorState` with the UI's state-update function. Use a streaming
text decoder if the serial API supplies byte arrays.

```js
let pending = '';

function finiteNumber(token) {
  const text = token.trim();
  if (text === '') return null;
  const value = Number(text);
  return Number.isFinite(value) ? value : null;
}

function onSensorTextChunk(chunk) {
  pending += chunk;
  let newline;
  while ((newline = pending.indexOf('\n')) !== -1) {
    const line = pending.slice(0, newline).trim();
    pending = pending.slice(newline + 1);
    const fields = line.split(',');

    // Handle RAW / DEVICE / INFO / READY / ERROR in separate handlers if needed.
    if (fields[0] !== 'DATA' || fields.length !== 15) continue;

    updateSensorState({
      uptimeMs: finiteNumber(fields[1]),
      motor1Torque: finiteNumber(fields[2]),
      motor2Torque: finiteNumber(fields[3]),
      motor1Thrust: finiteNumber(fields[4]),
      motor2Thrust: finiteNumber(fields[5]),
      motor1Rpm: finiteNumber(fields[6]),
      motor2Rpm: finiteNumber(fields[7]),
      motor1Voltage: finiteNumber(fields[8]),
      motor2Voltage: finiteNumber(fields[9]),
      motor1Current: finiteNumber(fields[10]),
      motor2Current: finiteNumber(fields[11]),
      receivedAtMs: Date.now(),
    });
  }
}

function formatVoltage(value) {
  return Number.isFinite(value) ? `${value.toFixed(3)} V` : 'Unavailable';
}
```

Merge the parsed measurements into existing UI state without overwriting
controller-owned fields. Clear the receive buffer when disconnecting or
reconnecting. Initialize readings as unavailable and mark them stale if no new
sensor `DATA` arrives for a suitable timeout, such as 2 seconds. Use host reception
time for freshness checks: Arduino uptime resets on reboot and wraps eventually.

## UI verification checklist

- Display two different voltages from indexes 8 and 9 independently.
- Accept a valid voltage when the other voltage or unrelated fields are `nan`.
- Display zero volts as a valid value.
- Do not clear measurements when `RAW`, `INFO`, or identity messages arrive.
- Handle a line split over several reads and several lines in one read.
- Indicate stale data when the sensor connection stops delivering `DATA`.

## I2C addressing versus UI mapping

I2C addresses are not transmitted in these records. Motor assignments are made
in firmware; the UI follows field positions. The firmware assigns motor 1 to `0x41` and motor 2 to `0x40`, matching the supplied protocol. Verify the physical address straps and wire each INA226 bus input to its corresponding motor supply. Both devices must have distinct addresses. INA226 addresses are in the range `0x40`–`0x4F`; `0x55` is not a supported INA226 address.

## Firmware verification

Host regression checks (mocked Arduino/I2C, no physical hardware):

```sh
g++ -std=c++17 -Wall -Wextra -I tests tests/voltage_test.cpp ForceReader.cpp -o /tmp/voltage_test
/tmp/voltage_test
```

Checks cover separate motor addresses and readings, 1.25 mV scaling, zero volts, independent startup/read failures, retry and recovery, per-window missing data, and DATA/RAW framing. Copy the sketch and headers into a `SensorMeasurements` sketch folder before compiling/uploading for an Uno with Arduino AVR core 1.8.6. No motor-controller firmware change is needed.
