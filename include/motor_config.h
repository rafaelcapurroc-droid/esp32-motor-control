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

// --- CONFIGURACIÓN PWM ---
#define PWM_FREQUENCY  20000
#define PWM_RESOLUTION 10          // 10 bits → valores 0-1023
#define PWM_CHANNEL    0
#define MIN_VELOCITY   0
#define MAX_VELOCITY   1023        // Rango real: 10-bit PWM

// --- SENSOR DE CORRIENTE ---
#define CS_VOLTAGE_PER_AMP  0.14f
#define CS_OFFSET_VOLTAGE   0.0f

// --- CONFIGURACIÓN MECÁNICA ---
#define MAGNETS_COUNT        6
#define DRIVE_ROLLER_DIAMETER_MM 36.0f  // Rodillo motriz de la cinta
#define PULLEY_DIAMETER_MM   60.0f      // Polea sensora (eje motriz)

// --- CONFIGURACIÓN DE TRANSMISIÓN ---
#define MOTOR_PULLEY_TEETH   20
#define AXIS_PULLEY_TEETH    60
#define TRANSMISSION_RATIO   ((float)MOTOR_PULLEY_TEETH / AXIS_PULLEY_TEETH)
#define MOTOR_RPM_MAX        2400.0f
#define AXIS_RPM_MAX         (MOTOR_RPM_MAX * TRANSMISSION_RATIO)

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