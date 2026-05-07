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

// --- SENSOR HALL (polea con imanes) ---
#define MAGNETS_COUNT        8
#define PULLEY_DIAMETER_MM   60

// --- RODILLO MOTRIZ ---
#define DRIVE_ROLLER_DIAMETER_MM 40

// --- CINTA ---
#define BELT_PERIMETER_MM 1150

// --- CONSTANTES DE VELOCIDAD ---
// Velocidad lineal de la cinta desde RPM:
// V(m/s) = π × D_rodillo(mm) × RPM / (60 × 1000)
// V(m/s) = 0.002094395 × RPM
#define M_S_PER_RPM 0.002094395f

// Para cálculo directo desde tiempo entre pulsos (μs):
// V(m/s) = (π × D_rodillo / MAGNETS_COUNT) / (T_us / 1e6) / 1000
// V(m/s) = 15707.963 / T_us
#define SPEED_FACTOR_MS_NUMERATOR 15707.963f

// --- CONFIGURACIÓN ENCODER ---
#define ENCODER_STEPS_PER_NOTCH 4
#define ENCODER_VELOCITY_STEP   10

// --- LÍMITES DE PROTECCIÓN ---
// Tiempo máximo sin pulso Hall antes de asumir velocidad 0 (ms)
#define HALL_TIMEOUT_MS 400

// Tiempo mínimo entre pulsos válidos (μs) — filtra rebotes
#define HALL_MIN_PULSE_US 100

#endif // MOTOR_CONFIG_H