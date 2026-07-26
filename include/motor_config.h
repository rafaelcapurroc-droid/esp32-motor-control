#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

// ============================================================
// PINES
// ============================================================
// --- Motor (driver VNH5019) ---
#define MOTOR_PWM_PIN 25   // PWM de velocidad (driver)
#define MOTOR_IN1_PIN 26   // Dirección de giro, entrada 1
#define MOTOR_IN2_PIN 27   // Dirección de giro, entrada 2
#define MOTOR_CS_PIN  35   // Salida analógica del sensor de corriente del driver

// --- Sensor Hall (velocidad) ---
#define HALL_SENSOR_PIN 33

// --- Encoder KY-040 ---
#define ENCODER_CLK_PIN 18
#define ENCODER_DT_PIN  19
#define ENCODER_SW_PIN  23  // Botón del encoder (confirmar setpoint / menú dev)

// --- LCD 16x2 I2C ---
#define LCD_SDA_PIN 21
#define LCD_SCL_PIN 22

// --- Parada de emergencia ---
#define EMERGENCY_STOP_PIN 13

// --- LED de estado (integrado en placa) ---
// Parpadea sin cliente WebSocket conectado; fijo ON con cliente conectado.
#define LED_BUILTIN_PIN 2

// ============================================================
// CONFIGURACIÓN PWM
// ============================================================
#define PWM_FREQUENCY  20000        // Frecuencia del PWM del driver (Hz)
#define PWM_RESOLUTION 10           // Resolución en bits → valores 0-1023
#define PWM_CHANNEL    0            // Canal LEDC usado (solo IDF < 5)
#define MAX_VELOCITY   1023         // PWM máximo posible (rango real de 10 bits)

// ============================================================
// SENSOR DE CORRIENTE
// ============================================================
#define CS_VOLTAGE_PER_AMP     0.14f  // Sensibilidad del sensor (V por A)
#define CS_OFFSET_VOLTAGE      0.0f  // Voltaje de salida del sensor con 0A (offset de cero)
#define CURRENT_READ_INTERVAL   5UL  // Cadencia de lectura (ms) → 200 Hz, ~3x el PID a 65Hz
#define CURRENT_ADC_SAMPLES       4  // Muestras ADC promediadas por lectura (<10% duty en el loop)

// ============================================================
// CONFIGURACIÓN MECÁNICA
// ============================================================
// Diente faltante: 5 imanes físicos montados en 6 posiciones equiespaciadas
// (una posición queda vacía). HALL_SLOTS_COUNT es la geometría (6, usada
// para convertir período→RPM/velocidad de los períodos normales);
// MAGNETS_COUNT es la cantidad real de imanes/pulsos por vuelta (5).
#define HALL_SLOTS_COUNT     6
#define MAGNETS_COUNT        5
#define DRIVE_ROLLER_DIAMETER_MM 38.5f  // Rodillo motriz de la cinta (mm)

// ============================================================
// HALL: TIMEOUT Y DEBOUNCE
// ============================================================
#define HALL_TIMEOUT_US      2000000UL  // Sin pulsos por más de esto (µs) → velocidad = 0 (2s)
#define HALL_DEBOUNCE_US     5000       // Piso de debounce (µs) — rechaza rebotes; permite hasta ~2 m/s
#define HALL_BUFFER_SIZE     6          // Tamaño del buffer circular de períodos (1 vuelta: 5 pulsos, 6 slots)

// Debounce dinámico: rechaza períodos menores a HALL_DEBOUNCE_RATIO × mediana
// actual (equivale a un salto de velocidad >2x entre pulsos, físicamente
// imposible en la cinta). HALL_DEBOUNCE_US es el piso en arranque/timeout;
// HALL_DEBOUNCE_MAX_US es el techo, y mantiene protección proporcional en
// todo el rango operativo 0.2–1.5 m/s (períodos ~13-100 ms).
#define HALL_DEBOUNCE_RATIO    0.5f     // Umbral dinámico = 50% de la mediana actual
#define HALL_DEBOUNCE_MAX_US   60000UL  // Techo del debounce dinámico (µs)

// ============================================================
// PID: GANANCIAS Y LÍMITES
// ============================================================
#define PID_KP_DEFAULT   50.0   // Ganancia proporcional inicial (editable en vivo desde el menú dev)
#define PID_KI_DEFAULT   50.0   // Ganancia integral inicial
#define PID_KD_DEFAULT    0.0   // Ganancia derivativa inicial (desactivada)

#define PID_OUTPUT_MIN   100          // PWM mínimo de salida — vence la fricción estática del motor
#define PID_OUTPUT_MAX   MAX_VELOCITY // PWM máximo de salida (1023)
#define PID_STARTUP_PWM  100          // Semilla del integrador al activar el PID (bumpless transfer)

// Cadencia del cómputo PID (ms). A 1.5 m/s (techo del rango operativo) el
// pulso Hall llega cada ~13.4ms — 15ms aprovecha casi cada pulso sin
// depender de timing perfecto del loop().
#define PID_TIMER_INTERVAL_MS   15UL

// ============================================================
// FILTRO DE VELOCIDAD (MEDIANA + EMA)
// ============================================================
#define FILTER_SIZE   3      // Cantidad de períodos Hall usados para la mediana
#define SMOOTH_ALPHA  1.0f   // Peso del EMA (1.0 = desactivado, 100% al valor nuevo)

// ============================================================
// TEMPORIZACIÓN GENERAL
// ============================================================
#define LCD_UPDATE_INTERVAL    300UL  // Refresco de pantalla LCD (ms)
#define DEBUG_INTERVAL        1000UL  // Log de depuración periódico por serial (ms)
#define WS_SEND_INTERVAL       200UL  // Cadencia de broadcast de estado por WebSocket (ms)

// ============================================================
// ENCODER KY-040: DEBOUNCE
// ============================================================
#define ENCODER_STABILITY_DELAY_MS 5    // Espera antes de aplicar un delta de giro (antirrebote)
#define ENCODER_BUTTON_DEBOUNCE_MS 50   // Antirrebote del botón del encoder

// ============================================================
// MENÚ DESARROLLADOR (edición de Kp/Ki/Kd en vivo desde el LCD)
// ============================================================
#define DEV_MENU_HOLD_MS 10000UL   // Tiempo manteniendo el botón para abrir/cerrar el menú (ms)
#define DEV_MENU_STEP_KP  0.1      // Incremento por click al editar Kp
#define DEV_MENU_STEP_KI  0.1      // Incremento por click al editar Ki
#define DEV_MENU_STEP_KD  0.1      // Incremento por click al editar Kd

// ============================================================
// PARADA DE EMERGENCIA
// ============================================================
#define EMERGENCY_DEBOUNCE_MS 100  // Antirrebote tras disparo de la emergencia (ms)

// ============================================================
// BUFFERS / LOGS
// ============================================================
#define PULSE_LOG_SIZE 32  // Tamaño del buffer de pulsos crudos para el log del WebSocket

#endif // MOTOR_CONFIG_H
