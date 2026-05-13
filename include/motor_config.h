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

// --- CONFIGURACIÓN MECÁNICA (Integrado desde Arduino Uno) ---
#define MAGNETS_COUNT        6
#define DRIVE_ROLLER_DIAMETER_MM 36.0f  // Rodillo motriz de la cinta
#define PULLEY_DIAMETER_MM   60.0f      // Polea sensora (eje motriz)

// --- CONFIGURACIÓN DE TRANSMISIÓN ---
#define MOTOR_PULLEY_TEETH   20
#define AXIS_PULLEY_TEETH    60
#define TRANSMISSION_RATIO   ((float)MOTOR_PULLEY_TEETH / AXIS_PULLEY_TEETH)
#define MOTOR_RPM_MAX        2400.0f
#define AXIS_RPM_MAX         (MOTOR_RPM_MAX * TRANSMISSION_RATIO)

// --- CÁLCULO DE CONSTANTES DE VELOCIDAD ---
// Perímetro del rodillo en metros: (PI * 36mm) / 1000 = 0.113097 m
#define DRIVE_ROLLER_PERIMETER_M (3.14159265f * DRIVE_ROLLER_DIAMETER_MM / 1000.0f)

// Factor para convertir Frecuencia (Hz) a m/s:
// V(m/s) = (Perimetro / Imanes) * Frecuencia(Hz)
// Factor = 0.113097 / 6 = 0.0188495
#define FACTOR_VELOCIDAD (DRIVE_ROLLER_PERIMETER_M / MAGNETS_COUNT)

// Para cálculo directo desde tiempo entre pulsos (μs):
// V(m/s) = (FACTOR_VELOCIDAD * 1,000,000) / periodoMicros
// Equivale a: 18849.5 / periodoMicros
#define SPEED_FACTOR_MS_NUMERATOR (FACTOR_VELOCIDAD * 1000000.0f)

// Relación RPM Eje Motriz:
// RPM = 60,000,000 / (periodoMicros * MAGNETS_COUNT)

// --- CONFIGURACIÓN ENCODER ---
#define ENCODER_STEPS_PER_NOTCH 4
#define ENCODER_VELOCITY_STEP   10

// --- LÍMITES DE PROTECCIÓN ---
// Tiempo máximo sin pulso Hall antes de asumir velocidad 0 (ms)
#define HALL_TIMEOUT_MS 1200

// Tiempo mínimo entre pulsos válidos (μs) — filtra rebotes
// Nota: En el código Arduino usabas 1000us (1ms). Aquí lo dejamos en 100us 
// para mayor resolución a altas velocidades, pero puedes subirlo a 1000 si hay mucho ruido.
#define HALL_MIN_PULSE_US 6000 

#endif // MOTOR_CONFIG_H