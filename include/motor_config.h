#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// --- PINES DEL MOTOR Y SENSORES ---
#define MOTOR_PWM_PIN 25
#define MOTOR_IN1_PIN 26
#define MOTOR_IN2_PIN 27
#define MOTOR_CS_PIN  35
#define HALL_SENSOR_PIN 33

// --- PINES ENCODER KY-040 ---
#define ENCODER_CLK_PIN 18
#define ENCODER_DT_PIN  19
#define ENCODER_SW_PIN  23

// --- PINES LCD 16x2 I2C ---
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22

// --- PIN DE PARADA DE EMERGENCIA ---
#define EMERGENCY_STOP_PIN 34

// --- CONFIGURACIÓN PWM ---
#define PWM_FREQUENCY  20000
#define PWM_RESOLUTION 11          // 11 bits → valores 0-2047
#define PWM_CHANNEL    0
#define MIN_VELOCITY   0
#define MAX_VELOCITY   2047        // Rango real: 11-bit PWM

// --- WIFI Y RED ---
#define WIFI_SSID        "ESP32_Motor_Control"
#define WIFI_AP_PASSWORD "12345678"

// --- TIEMPOS DE CONTROL ---
#define WS_SEND_INTERVAL       200
#define CURRENT_READ_INTERVAL  50
#define DEBUG_INTERVAL         1000
#define ENCODER_DEBOUNCE_MS    5
#define BUTTON_DEBOUNCE_MS     200
#define EMERGENCY_DEBOUNCE_MS  100
#define LCD_UPDATE_INTERVAL    200

// --- PINES DE SISTEMA ---
#define LED_BUILTIN_PIN 2

// --- SENSOR DE CORRIENTE ---
#define CS_VOLTAGE_PER_AMP  0.14f
#define CS_OFFSET_VOLTAGE   0.0f

// --- CONFIGURACIÓN MECÁNICA ---
#define MAGNETS_COUNT        6
#define DRIVE_ROLLER_DIAMETER_MM 36.0f  // Rodillo motriz de la cinta
#define PULLEY_DIAMETER_MM   60.0f      // Polea sensora (eje motriz)

#define HALL_CONST_RPM   (60000000.0f / MAGNETS_COUNT)
#define HALL_CONST_SPEED ((3.14159265f * DRIVE_ROLLER_DIAMETER_MM * 1000.0f) / MAGNETS_COUNT)

// --- PID ---
#define PID_KP 2.5f
#define PID_KI 5.0f
#define PID_KD 0.2f
#define MAX_SPEED_MS 5.0f

// --- CONFIGURACIÓN ENCODER ---
#define ENCODER_STEPS_PER_NOTCH 4
#define ENCODER_VELOCITY_STEP   10

// --- LÍMITES DE PROTECCIÓN ---
// Tiempo máximo sin pulso Hall antes de asumir velocidad 0 (µs)
#define HALL_TIMEOUT_US      2000000UL   // 2 segundos (igual que Arduino Uno)

// Debounce mínimo entre pulsos válidos (µs) — igual que Arduino Uno
#define HALL_DEBOUNCE_US     200

// Tamaño del buffer circular de períodos (1 vuelta completa con 6 imanes)
#define HALL_BUFFER_SIZE     6

#endif // MOTOR_CONFIG_H