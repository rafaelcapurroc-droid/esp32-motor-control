---
name: ESP32 WiFi Motor Control
description: "Use when working on ESP32 WiFi web server for DC motor control with VNH5019 driver. Covers build commands, WiFi setup, web server API, and troubleshooting."
---

# ESP32 WiFi Motor Control Development Guide

## Main Project: WiFi Web Server Motor Control System

This workspace contains an **ESP32-based WiFi web server** for controlling a DC motor via VNH5019 Pololu driver with real-time monitoring via browser.

### Architecture
```
Browser (http://192.168.4.1)
         ↓ HTTP/WebSocket
   WiFi Network (2.4GHz)
         ↓
   ESP32 Web Server (80)
         ├─ HTML/CSS/JS Page
         ├─ WebSocket (/ws) - real-time data
         └─ REST API (/api/*)
         ↓
   Motor Controller
         ├─ PWM (GPIO 25)
         ├─ Direction (GPIO 26/27)
         └─ Current Sense (GPIO 35)
         ↓
   VNH5019 Driver + Motor DC
```

### Key Features
- ✅ No app required - works in any browser
- ✅ Real-time graphs (velocity, current, power)
- ✅ Mobile responsive interface
- ✅ WebSocket for low-latency updates
- ✅ Works on iOS and Android simultaneously

### Key Files
- **src/main.cpp**: WiFi server + motor control firmware
- **include/web_pages.h**: HTML/CSS/JS embedded
- **include/wifi_config.h**: WiFi SSID/password configuration
- **include/motor_config.h**: GPIO pin definitions
- **README.md**: Quick start & full documentation
- **README_WIFI.md**: Detailed WiFi setup guide

### WiFi Configuration
Edit `include/wifi_config.h`:

```cpp
#define WIFI_SSID "ESP32_Motor_Control"    // Network name
#define WIFI_PASSWORD ""                    // Leave empty for open
#define AP_SSID "ESP32_Motor_Control"       // Access Point name
#define AP_PASSWORD "12345678"              // AP password
```

### Hardware Setup
```
ESP32 GPIO 25 → VNH5019 PWM
ESP32 GPIO 26 → VNH5019 IN1
ESP32 GPIO 27 → VNH5019 IN2
ESP32 GPIO 35 → VNH5019 CS (current sense ADC)
GND ────────→ VNH5019 GND
```

### Web Interface

**URL**: `http://192.168.4.1` (or router IP if connected to WiFi)

**Endpoints**:
- GET `/` → HTML page
- GET `/style.css` → CSS styling
- GET `/script.js` → JavaScript code
- GET `/api/status` → JSON motor status
- GET `/api/history` → Historical data for graphs
- POST `/api/velocity` → Set motor speed
- POST `/api/direction` → Set direction
- WebSocket `/ws` → Real-time updates

### Development Workflow for Web Server

1. **Modify firmware** in `src/main.cpp`
   - Motor control: `updateMotor()`, `readMotorCurrent()`
   - WebSocket handling: `onWebSocketEvent()`
   - HTTP endpoints: `setupHTTPEndpoints()`

2. **Modify web interface** in `include/web_pages.h`
   - HTML layout: `HTML_PAGE` constant
   - CSS styling: `CSS_STYLE` constant
   - JavaScript logic: `JS_SCRIPT` constant

3. **Configure hardware** in `include/motor_config.h`
   - GPIO pins, PWM frequency, current limits

4. **Configure WiFi** in `include/wifi_config.h`
   - Network name, password, access point settings

5. **Build and Test**:
   ```powershell
   pio run                        # Compile
   pio run --target upload       # Upload to ESP32
   pio device monitor            # View serial output
   ```

6. **Access from browser**:
   - Open `http://192.168.4.1`
   - Test slider → check motor responds
   - Monitor WebSocket in browser console (F12)
   - Check serial output for debug messages

### Essential Commands

```powershell
# Build project
pio run

# Upload to ESP32
pio run --target upload

# Monitor serial (115200 baud)
pio device monitor --baud 115200

# Monitor with decoder
pio device monitor -f esp32_exception_decoder

# Clean build
pio run --target clean

# Check devices
pio device list
```

### Web Server Data Format

**WebSocket message (Client → Server)**:
```json
{"velocity": 150}
{"direction": 1}
```

**WebSocket message (Server → Client, every 100ms)**:
```json
{
  "velocity": 150,
  "direction": 1,
  "current": 2.34,
  "timestamp": 5234
}
```

### Common Tasks

**Adjust motor speed**:
- Move slider on web page → JavaScript sends `{"velocity": X}` via WebSocket
- ESP32 receives in `onWebSocketEvent()` → updates `motorVelocity`
- Next loop: `updateMotor()` applies PWM to GPIO 25

**Change motor direction**:
- Click Adelante/Atrás button → sends `{"direction": 1/-1}`
- ESP32 updates GPIO 26/27 in `updateMotor()`

**Read current sensor**:
- `readMotorCurrent()` converts ADC value to Amperes
- Sent to client every 100ms via WebSocket
- JavaScript receives and updates graphs in real-time

**Troubleshoot current readings**:
- Calibrate CS pin offset and scale in `readMotorCurrent()`
- Compare multimeter reading vs web server display
- Adjust formula: `(voltage - OFFSET) / SCALE`

### Performance Characteristics

| Parameter | Value |
|-----------|-------|
| Update Frequency | 100ms (10 Hz) |
| WebSocket Latency | <50ms |
| Control Response Time | ~10ms to motor |
| Simultaneous Clients | 5-10 typical |
| Memory Usage | ~60KB RAM |
| WiFi Range | 30m interior typical |

### Serial Output Examples

**Startup**:
```
[WiFi] Starting WiFi...
[WiFi] Access Point: ESP32_Motor_Control
[WiFi] AP IP: 192.168.4.1
[HTTP] Web server started on port 80
```

**During operation**:
```
[WS] Client 1 connected
[WS] Velocity: 200
[DATA] V:200 D:1 I:2.34A T:5234
[WS] Client 1 disconnected
```

### Debugging Tips

- **Browser console**: F12 → Console → See WebSocket messages
- **Serial monitor**: `pio device monitor` → See ESP32 logs
- **Network**: Chrome DevTools → Network → Inspect WebSocket frames
- **Current calibration**: Measure with multimeter, compare to web display

---

## Development Workflow Summary

1. **Edit code** in `src/main.cpp` or `include/web_pages.h`
2. **Build**: `pio run`
3. **Upload**: `pio run --target upload` (with ESP32 connected)
4. **Test**: Open `http://192.168.4.1` in browser
5. **Debug**: `pio device monitor` (F12 in browser for console)
6. **Repeat** until desired behavior is achieved

---

**Use this guide** when:
- Adding new web interface features
- Modifying motor control logic
- Debugging WebSocket communication
- Adjusting WiFi settings
- Calibrating current sensor
- Optimizing performance
- Adding new API endpoints
- Testing with multiple clients
