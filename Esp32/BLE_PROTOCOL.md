# BLE Motor Control Protocol

## Overview
This document describes the BLE GATT structure for controlling the motor and reading sensor data.

## BLE Service Structure

### Service UUID
- **UUID**: `12345678-1234-5678-1234-56789abcdef0`
- **Device Name**: `ESP32_MotorControl`

---

## Characteristics

### 1. Velocity Control (Read/Write/Notify)
- **UUID**: `87654321-4321-8765-4321-fedcba987654`
- **Properties**: READ, WRITE, NOTIFY
- **Data Format**: JSON
- **Example Write**:
  ```json
  {
    "velocity": 200
  }
  ```
- **Range**: 0-255 (0% to 100% speed)
- **Response**: Same JSON echoed back on read

---

### 2. Direction Control (Read/Write/Notify)
- **UUID**: `abcdef01-2345-6789-abcd-ef0123456789`
- **Properties**: READ, WRITE, NOTIFY
- **Data Format**: JSON
- **Example Write**:
  ```json
  {
    "direction": 1
  }
  ```
- **Values**:
  - `1` = Forward (IN1 HIGH, IN2 LOW)
  - `-1` = Reverse (IN1 LOW, IN2 HIGH)
- **Response**: Same JSON echoed back on read

---

### 3. Current Sense (Read/Notify)
- **UUID**: `11111111-2222-3333-4444-555555555555`
- **Properties**: READ, NOTIFY
- **Data Format**: JSON (sent every 100ms when connected)
- **Example Data**:
  ```json
  {
    "current": 2.34,
    "timestamp": 1234567890
  }
  ```
- **Current**: in Amperes (float)
- **Timestamp**: milliseconds since ESP32 boot

---

### 4. Motor Status (Read/Notify)
- **UUID**: `22222222-3333-4444-5555-666666666666`
- **Properties**: READ, NOTIFY
- **Data Format**: JSON (sent every 100ms when connected)
- **Example Data**:
  ```json
  {
    "velocity": 200,
    "direction": 1,
    "current": 2.34,
    "timestamp": 1234567890
  }
  ```
- **Provides**: Complete motor state snapshot

---

## Data Flow

### Setting Motor Speed & Direction
1. App sends JSON to **Velocity Characteristic** (write)
2. ESP32 receives, parses, applies PWM
3. ESP32 updates internal state
4. Status notification sent with new values

### Reading Sensor Data
1. ESP32 continuously reads current sensor (every 100ms)
2. **Current Characteristic** notified with current reading
3. **Status Characteristic** notified with full status
4. App receives and displays in real-time graphs

---

## Connection Flow (Mobile App)

```
1. Scan for "ESP32_MotorControl"
2. Connect to BLE device
3. Discover Service (UUID: 12345678...)
4. Enable notifications on Current & Status characteristics
5. Write to Velocity/Direction as needed
6. Receive data via notifications
```

---

## Serial Monitor Output
When connected, the ESP32 prints:
```
[BLE] Device connected
[MOTOR] Velocity set to: 150
[DATA] V:150 D:1 I:2.34A T:1234567890
[BLE] Device disconnected
```

---

## JSON Parsing in Mobile App

### Example: Flutter/Dart
```dart
import 'dart:convert';

// Parse status update
Map<String, dynamic> status = jsonDecode(statusString);
int velocity = status['velocity'];
int direction = status['direction'];
double current = status['current'];
```

### Example: React Native
```javascript
const status = JSON.parse(statusString);
const { velocity, direction, current } = status;
```

---

## Calibration Notes

### Current Sensor (CS Pin)
- **Voltage Output**: 1V per 1A (nominal)
- **Offset**: ~0.5V at zero current
- **Formula**: `Current = (Voltage - 0.5V) / 1.0`
- **Adjustment**: If readings are inaccurate, modify `CS_OFFSET_VOLTAGE` and `CS_VOLTAGE_PER_AMP` in `motor_config.h`

### Example Calibration
```
Measured: 0.0A → Voltage: 0.48V
Measured: 1.0A → Voltage: 1.48V
Measured: 2.0A → Voltage: 2.48V
```

---

## Troubleshooting

### BLE Won't Connect
- Check if ESP32 is advertising (`ESP32_MotorControl`)
- Rebuild and re-upload with `pio run --target upload`
- Check serial monitor: should see `[BLE] Server started and advertising`

### No Data Received
- Ensure notifications are enabled on characteristics
- Check if device is actually connected (serial: `[BLE] Device connected`)
- Verify filter in app (data sent every 100ms)

### Motor Not Moving
- Check power supply to VNH5019
- Verify motor wiring (M1/M2)
- Check GPIO pins match configuration in `motor_config.h`
- Test with serial commands first

### Current Readings Inaccurate
- Calibrate CS pin voltage
- Check if motor is drawing expected current
- Verify ADC pin (GPIO 35) is not used by Wi-Fi/BLE

---

## Next Steps
1. Build and upload to ESP32: `pio run --target upload`
2. Monitor serial output: `pio device monitor`
3. Create mobile app (Flutter or React Native)
4. Connect mobile app to BLE device
5. Test velocity/direction control
6. Verify current readings
7. Build real-time graphs for velocity/current vs time
