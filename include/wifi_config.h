#ifndef WIFI_CONFIG_H
#define WIFI_CONFIG_H

// ============================================================================
// WiFi Configuration
// ============================================================================

// Name of your WiFi network (SSID)
// Change this to your actual network name
#define WIFI_SSID "ESP32_Motor_Control"

// WiFi password
// Leave empty ("") for open network without password
// For secure network, set your WiFi password here
#define WIFI_PASSWORD ""

// ============================================================================
// Access Point Configuration
// ============================================================================

// When ESP32 can't connect to WiFi, it creates its own Access Point (hotspot)
// You can connect to this AP from your phone/computer

#define AP_SSID "ESP32_Motor_Control"      // Access Point name
#define AP_PASSWORD "12345678"              // Access Point password (min 8 chars)

// ============================================================================
// How to Connect
// ============================================================================

// OPTION 1: ESP32 Creates WiFi Hotspot (No external WiFi needed)
// ─────────────────────────────────────────────────────────
// 1. Upload code to ESP32
// 2. On your phone/computer, look for WiFi network: "ESP32_Motor_Control"
// 3. Connect with password: "12345678"
// 4. Open browser and go to: http://192.168.4.1
// 5. Control motor from web interface!

// OPTION 2: Connect ESP32 to Your Home WiFi
// ─────────────────────────────────────────────────────────
// 1. Edit WIFI_SSID above with your WiFi network name
// 2. Edit WIFI_PASSWORD above with your WiFi password
// 3. Upload code to ESP32
// 4. Check serial monitor for IP address: "IP Address: 192.168.x.x"
// 5. Open browser and go to: http://192.168.x.x
// 6. Control motor from web interface!

// OPTION 3: Use mDNS (Recommended)
// ─────────────────────────────────────────────────────────
// If connected to WiFi, you can also access via hostname:
// http://esp32-motor.local
// (requires mDNS available in your network)

// ============================================================================
// Debug Information
// ============================================================================

// After uploading, open Serial Monitor (115200 baud) and you'll see:
// 
// If connected to WiFi:
// [WiFi] Connected to: Your_Network_Name
// [WiFi] IP Address: 192.168.1.100
// 
// If running as Access Point:
// [WiFi] Access Point: ESP32_Motor_Control
// [WiFi] AP IP: 192.168.4.1

#endif // WIFI_CONFIG_H
