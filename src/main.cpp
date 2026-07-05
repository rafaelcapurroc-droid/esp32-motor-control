// ============================================================
//  TROTADORA PARA RATAS — Control PID v9.11.0
//  Plataforma: ESP32 + Driver VNH5019 + Sensor Hall + KY-040
//
//  Cambios v9.10.1 → v9.11.0:
//    - NUEVO: Timer fallback en loop() — PID garantizado mínimo 20 Hz
//      independientemente de la frecuencia de pulsos Hall.
//      Resuelve respuesta lenta ante frenado brusco de la cinta.
//  Cambios v9.10 → v9.10.1:
//    - CORREGIDO: firstControlCall = false en activarPID()
//    - CORREGIDO: pidLastError inicializado con error real
//    - CORREGIDO: Anti-windup con rango simétrico ±(PWM_MAX - PWM_MIN)/Ki
//    - CORREGIDO: Protección Ki > 0 para evitar división por cero
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <algorithm>
#include "motor_config.h"
#include <ESP32Encoder.h>
#include <PID_v1.h>

// ============================================================
// 1. RED / SERVIDOR
// ============================================================
const char* ssid       = "ESP32_Motor_Control";
const char* apPassword = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

bool          isWebConnected  = false;
unsigned long lastWsSendTime  = 0;
const unsigned long WS_SEND_INTERVAL = 200;

// ============================================================
// 2. CONSTANTES PWM / FÍSICAS
// ============================================================
// Debounce dinámico: rechaza períodos menores a HALL_DEBOUNCE_RATIO × mediana
// actual (equivale a un salto de velocidad >2x entre pulsos, físicamente
// imposible en la cinta). HALL_DEBOUNCE_US (5 ms, motor_config.h) es el piso
// en arranque/timeout; el techo mantiene protección proporcional en todo el
// rango operativo 0.2–1.5 m/s (períodos ~13–100 ms).
#define HALL_DEBOUNCE_RATIO    0.5f
#define HALL_DEBOUNCE_MAX_US   60000UL
static volatile uint32_t hallMinPeriodUs = HALL_DEBOUNCE_US;

// Límites de salida del PID
#define PID_OUTPUT_MIN   100          // PWM mínimo para vencer fricción estática
#define PID_OUTPUT_MAX   MAX_VELOCITY // 1023

static_assert(HALL_BUFFER_SIZE > 1, "HALL_BUFFER_SIZE debe ser > 1");

static const float HALL_CONST_RPM   = 60000000.0f / (float)MAGNETS_COUNT;
static const float HALL_CONST_SPEED = (3.14159265f * DRIVE_ROLLER_DIAMETER_MM * 1000.0f)
                                       / (float)MAGNETS_COUNT;

// ============================================================
// 3. TEMPORIZACIÓN
// ============================================================
#define PID_TIMER_INTERVAL_MS   25UL   // cadencia del loop PID (40 Hz mínimo aunque fallen pulsos Hall)
#define LCD_UPDATE_INTERVAL    300UL   // refresco de pantalla LCD
#define DEBUG_INTERVAL        1000UL   // log de depuración por serial
#define CURRENT_READ_INTERVAL 50UL    // lectura del sensor de corriente (20 Hz)

// ============================================================
// 4. FILTRO DE MEDIANA + EMA
// ============================================================
#define FILTER_SIZE   3
#define SMOOTH_ALPHA  1.0f  // EMA desactivada (peso 100% al valor nuevo)

static uint32_t filterBuffer[FILTER_SIZE];
static uint8_t  filterIndex = 0;
static uint8_t  filterCount = 0;

static float filteredRPM_Last = 0.0f;
static float filteredVel_Last = 0.0f;

// --- Velocidad para display (LCD) ---
// Promedio deslizante de los últimos MAGNETS_COUNT períodos (= 1 vuelta
// completa): cancela el error por espaciado desigual de imanes y actualiza
// en cada pulso. Solo para mostrar; el PID sigue usando la mediana.
static uint32_t revPeriodBuffer[MAGNETS_COUNT] = {0};
static uint8_t  revBufIndex    = 0;
static uint8_t  revBufCount    = 0;
static uint32_t revPeriodSum   = 0;
static float    displaySpeedMs = 0.0f;

// ============================================================
// 5. VARIABLES DE VELOCIDAD / PID
// ============================================================
float currentRPM           = 0.0f;
float currentLinearSpeedMs = 0.0f;
float setpointSpeed        = 0.0f;   // Setpoint activo (confirmado)
float pendingSetpoint      = 0.0f;   // Setpoint temporal del encoder
float pidOutput            = 0.0f;
bool  setpointPending      = false;  // Hay un setpoint pendiente de confirmar

// --- CSV LOG ---
bool csvLogActive = false;

// --- MONITOR HALL ---
static uint8_t magnetCount     = 0;     // cuenta imanes en la vuelta actual (0..MAGNETS_COUNT-1)
static bool    hallDebugActive = false; // activado con "HALL ON" por serial

// --- PID Brett Beauregard ---
double Kp = 150.0;
double Ki = 150.0;
double Kd = 0.0;

double pidIn  = 0.0;  // Input  → currentLinearSpeedMs
double pidOut = 0.0;  // Output → PWM
double pidSp  = 0.0;  // Setpoint

PID myPID(&pidIn, &pidOut, &pidSp, Kp, Ki, Kd, DIRECT);

float pidIntegral = 0.0f;          // NO USADO — reservado para compatibilidad WS
unsigned long lastControlMicros = 0; // para el fallback timer de loop()

// Feed-Forward cache (tabla empírica conservada, no activa)
float lastFF_PWM = 0.0f;

// ============================================================
// 6. BUFFER CIRCULAR HALL
// ============================================================
static volatile uint32_t hallPeriodBuffer[HALL_BUFFER_SIZE] = {0};
static volatile uint8_t  hallBufferHead  = 0;
static volatile uint8_t  hallBufferCount = 0;
static volatile uint32_t lastHallMicros  = 0;
static uint8_t           lastProcessedHead = 0;

// ============================================================
// 6b. LOG CRUDO IMÁN POR IMÁN (para CSV, no usado por el PID)
// ============================================================
#define PULSE_LOG_SIZE 32

struct PulseSample {
  uint32_t periodo_us;  // período crudo entre este imán y el anterior
  float    rpm;         // RPM instantánea calculada de ese período
  float    vel_m_s;     // velocidad lineal instantánea de ese período
};

static PulseSample pulseLogBuffer[PULSE_LOG_SIZE];
static uint8_t     pulseLogCount    = 0;
static bool        pulseLogOverflow = false;

// ============================================================
// 7. ENCODER KY-040
// ============================================================
ESP32Encoder encoder;
int           lastStableCount    = 0;
unsigned long lastStableTime     = 0;
const unsigned long ENCODER_STABILITY_DELAY_MS = 5;
int           displayEncoderCount = 0;

volatile bool encoderButtonPressed  = false;
unsigned long lastButtonDebounce    = 0;
const unsigned long ENCODER_BUTTON_DEBOUNCE_MS = 50;

// ============================================================
// 8. PARADA DE EMERGENCIA
// ============================================================
volatile bool emergencyStopTriggered = false;
bool          emergencyStopPending   = false;
unsigned long lastEmergencyDebounce  = 0;
const unsigned long EMERGENCY_DEBOUNCE_MS = 100;

// ============================================================
// 9. CORRIENTE
// ============================================================
float         cachedCurrent       = 0.0f;
unsigned long lastCurrentReadTime = 0;

// ============================================================
// 10. LCD
// ============================================================
LiquidCrystal_I2C* lcd        = nullptr;
unsigned long      lastLcdUpdate = 0;

// ============================================================
// 11. SERIAL
// ============================================================
static char    serialBuf[64];
static uint8_t serialIdx = 0;
unsigned long  lastDebugTime = 0;

// ============================================================
// 12. LED
// ============================================================
#define LED_BUILTIN_PIN 2
unsigned long lastBlink = 0;

// ============================================================
// 13. DECLARACIONES ANTICIPADAS
// ============================================================
void broadcastState();
void motorStop();
void computePID();

// ============================================================
// 14. IMPLEMENTACIÓN DE FUNCIONES
// ============================================================

// applyPWM: escribe el duty cycle (0..MAX_VELOCITY) en el canal PWM del driver
void applyPWM(int pwmVal) {
  pwmVal = constrain(pwmVal, 0, MAX_VELOCITY);
#if ESP_IDF_VERSION_MAJOR >= 5
  ledcWrite(MOTOR_PWM_PIN, (uint32_t)pwmVal);
#else
  ledcWrite(PWM_CHANNEL, (uint32_t)pwmVal);
#endif
}

// configurarMotorReversa: fija los pines IN1/IN2 del driver para girar en el
// sentido usado por la cinta (única dirección que usa este proyecto)
void configurarMotorReversa() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

// motorParadaLibre: pone ambos pines IN1/IN2 en LOW (coast / parada libre, sin frenado activo)
void motorParadaLibre() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

// ---------------------
// motorStop: apaga motor, reset PID manual
// ---------------------
void motorStop() {
  setpointSpeed   = 0.0f;
  pendingSetpoint = 0.0f;
  setpointPending = false;
  pidOutput       = 0.0f;
  applyPWM(0);
  motorParadaLibre();

  // Resetear PID (MANUAL + Output=0 → próximo SetMode(AUTOMATIC) arranca desde 0)
  myPID.SetMode(MANUAL);
  pidOut      = 0.0;
  pidIntegral = 0.0f;
  Serial.println("[PID] Motor detenido — PID reset");
}

// ---------------------
// activarPID: configura setpoint y arranca el PID
// ---------------------
static void activarPID(float sp) {
  pidSp = sp;
  pidIn = currentLinearSpeedMs;
  myPID.SetMode(AUTOMATIC);
  lastControlMicros = micros();

  Serial.printf("[PID] Activado — SP: %.3f m/s\n", sp);
}

// ---------------------
// computePID: delega en PID de Brett Beauregard (maneja timing internamente)
// ---------------------
void computePID() {
  if (setpointSpeed == 0.0f) {
    pidOutput = 0.0f;
    return;
  }

  pidIn = currentLinearSpeedMs;
  pidSp = setpointSpeed;

  if (myPID.Compute()) {
    pidOutput = (float)pidOut;
    applyPWM((int)round(pidOutput));
    lastControlMicros = micros();  // actualiza el fallback timer
  }
}

// ---------------------
// confirmarSetpoint: aplica setpoint pendiente del encoder
// ---------------------
void confirmarSetpoint() {
  if (!setpointPending) return;

  setpointSpeed = constrain(pendingSetpoint, 0.0f, 2.0f);
  setpointPending = false;

  if (setpointSpeed == 0.0f) {
    motorStop();
    Serial.println("[ENCODER] Setpoint confirmado: 0 — Motor detenido");
  } else {
    configurarMotorReversa();
    activarPID(setpointSpeed);
    Serial.printf("[ENCODER] Setpoint confirmado: %.3f m/s\n", setpointSpeed);
  }

  broadcastState();
}

// ---------------------
// procesarEntradasEncoder
// ---------------------
void procesarEntradasEncoder() {
  int rawCount      = encoder.getCount();
  unsigned long ahora = millis();

  if (rawCount != lastStableCount) {
    lastStableTime  = ahora;
    lastStableCount = rawCount;
  }

  if ((ahora - lastStableTime) >= ENCODER_STABILITY_DELAY_MS) {
    int newDisplayCount = rawCount / 2;

    if (newDisplayCount != displayEncoderCount) {
      int delta = newDisplayCount - displayEncoderCount;
      displayEncoderCount = newDisplayCount;

      pendingSetpoint += delta * 0.05f;
      pendingSetpoint  = constrain(pendingSetpoint, 0.0f, 2.0f);
      setpointPending  = true;

      Serial.printf("[ENCODER] Setpoint temporal: %.3f m/s (delta: %d)\n",
                    pendingSetpoint, delta);
      broadcastState();
    }
  }

  // Botón: confirmar setpoint pendiente
  if (encoderButtonPressed) {
    unsigned long ahoraMs = millis();
    if (ahoraMs - lastButtonDebounce > ENCODER_BUTTON_DEBOUNCE_MS) {
      confirmarSetpoint();
      lastButtonDebounce = ahoraMs;
    }
    encoderButtonPressed = false;
  }
}

// ============================================================
// 15. ISRs
// ============================================================
// ISR_SensorHall: en cada flanco del sensor Hall mide el período respecto al
// pulso anterior y, si supera el debounce dinámico, lo guarda en el buffer circular
void IRAM_ATTR ISR_SensorHall() {
  uint32_t ahora = micros();

  if (lastHallMicros == 0) {
    lastHallMicros = ahora;
    return;
  }

  uint32_t periodo = ahora - lastHallMicros;

  if (periodo >= hallMinPeriodUs) {
    lastHallMicros = ahora;
    hallPeriodBuffer[hallBufferHead] = periodo;
    hallBufferHead = (hallBufferHead + 1) % HALL_BUFFER_SIZE;
    if (hallBufferCount < HALL_BUFFER_SIZE) hallBufferCount++;
  }
  // Pulso espurio: NO se avanza lastHallMicros, así el próximo flanco real
  // mide el período completo desde el último pulso válido (evita que el
  // rebote parta el período en dos fragmentos cortos).
}

// ISR_EncoderBoton: marca el botón del encoder como presionado (con debounce por tiempo)
void IRAM_ATTR ISR_EncoderBoton() {
  if (millis() - lastButtonDebounce > ENCODER_BUTTON_DEBOUNCE_MS) {
    encoderButtonPressed = true;
  }
}

// ISR_EmergencyStop: levanta la bandera de parada de emergencia; el loop() la procesa
void IRAM_ATTR ISR_EmergencyStop() {
  emergencyStopTriggered = true;
}

// ============================================================
// 16. CÁLCULO DE VELOCIDAD + PID EVENT-DRIVEN
// ============================================================
// getMedian: copia y ordena hasta `size` muestras del buffer para devolver la mediana
static float getMedian(uint32_t* arr, uint8_t size, uint8_t count) {
  uint32_t temp[FILTER_SIZE];
  uint8_t  n = (count < size) ? count : size;
  if (n == 0) return 0.0f;
  memcpy(temp, arr, n * sizeof(uint32_t));
  std::sort(temp, temp + n);
  return (float)temp[n / 2];
}

// Empuja una muestra cruda (sin filtrar) al log imán por imán.
static void logPulsoCrudo(uint32_t periodo_us, float rpm, float vel_m_s) {
  if (pulseLogCount >= PULSE_LOG_SIZE) {
    pulseLogOverflow = true;
    return;
  }
  pulseLogBuffer[pulseLogCount].periodo_us = periodo_us;
  pulseLogBuffer[pulseLogCount].rpm        = rpm;
  pulseLogBuffer[pulseLogCount].vel_m_s    = vel_m_s;
  pulseLogCount++;
}

// calculateSpeed: procesa los pulsos Hall nuevos desde el último ciclo — actualiza
// RPM/velocidad filtrada (mediana), la velocidad de display, el log crudo para el
// WebSocket, maneja el timeout (sin pulsos = velocidad 0) y dispara computePID()
static void calculateSpeed() {
  uint32_t periods[HALL_BUFFER_SIZE];
  uint8_t  currentHead;
  uint32_t ultimoPulso;

  portDISABLE_INTERRUPTS();
  memcpy(periods, (const void*)hallPeriodBuffer, sizeof(periods));
  currentHead  = hallBufferHead;
  ultimoPulso  = lastHallMicros;
  portENABLE_INTERRUPTS();

  bool timeout = (ultimoPulso == 0) || ((micros() - ultimoPulso) > HALL_TIMEOUT_US);

  if (timeout) {
    currentRPM           = 0.0f;
    currentLinearSpeedMs = 0.0f;
    filteredRPM_Last     = 0.0f;
    filteredVel_Last     = 0.0f;

    memset(filterBuffer, 0, sizeof(filterBuffer));
    filterCount = 0;
    filterIndex = 0;

    memset(revPeriodBuffer, 0, sizeof(revPeriodBuffer));
    revBufIndex    = 0;
    revBufCount    = 0;
    revPeriodSum   = 0;
    displaySpeedMs = 0.0f;

    portDISABLE_INTERRUPTS();
    hallBufferCount   = 0;
    hallBufferHead    = 0;
    lastProcessedHead = 0;
    lastHallMicros    = 0;  // reset: el próximo imán será la referencia, el siguiente se mide
    hallMinPeriodUs   = HALL_DEBOUNCE_US;  // vuelve al piso hasta tener mediana nueva
    memset((void*)hallPeriodBuffer, 0, sizeof(hallPeriodBuffer));
    portENABLE_INTERRUPTS();

    // Tras timeout: Brett PID retoma en el próximo Compute() con Input~0
    return;
  }

  uint8_t prevProcessedHead = lastProcessedHead;
  uint8_t newPulses = (currentHead - prevProcessedHead + HALL_BUFFER_SIZE) % HALL_BUFFER_SIZE;
  lastProcessedHead = currentHead;

  if (newPulses == 0) return;

  // Acumular períodos crudos válidos en array local para CSV post-EMA
  uint32_t rawPeriods[HALL_BUFFER_SIZE];
  uint8_t  validCount = 0;

  // Procesar cada nuevo pulso
  bool revCompleted = false;
  for (uint8_t i = 0; i < newPulses; i++) {
    uint8_t  idx = (prevProcessedHead + i) % HALL_BUFFER_SIZE;
    uint32_t p   = periods[idx];
    if (p > HALL_DEBOUNCE_US && p < HALL_TIMEOUT_US) {
      filterBuffer[filterIndex] = p;
      filterIndex = (filterIndex + 1) % FILTER_SIZE;
      if (filterCount < FILTER_SIZE) filterCount++;

      rawPeriods[validCount++] = p;

      // Ventana deslizante de 1 vuelta para la velocidad de display
      revPeriodSum -= revPeriodBuffer[revBufIndex];
      revPeriodBuffer[revBufIndex] = p;
      revPeriodSum += p;
      revBufIndex = (revBufIndex + 1) % MAGNETS_COUNT;
      if (revBufCount < MAGNETS_COUNT) revBufCount++;

      // Log WebSocket (imán por imán)
      float rawRpmSample = HALL_CONST_RPM   / (float)p;
      float rawVelSample = HALL_CONST_SPEED / (float)p;
      logPulsoCrudo(p, rawRpmSample, rawVelSample);

      // Monitor de imanes
      magnetCount++;
      if (hallDebugActive) {
        Serial.printf("[HALL] %u/%u  p=%luus  v=%.3fm/s\n",
                      magnetCount, (uint8_t)MAGNETS_COUNT,
                      (unsigned long)p, HALL_CONST_SPEED / (float)p);
      }
      if (magnetCount >= MAGNETS_COUNT) {
        magnetCount  = 0;
        revCompleted = true;
      }
    }
  }

  if (filterCount == 0) return;

  // Calcular velocidad filtrada (mediana + EMA)
  float periodoMediano = getMedian(filterBuffer, FILTER_SIZE, filterCount);

  // Actualizar umbral del debounce dinámico (leído por la ISR)
  uint32_t nuevoUmbral = (uint32_t)(HALL_DEBOUNCE_RATIO * periodoMediano);
  if (nuevoUmbral < HALL_DEBOUNCE_US)     nuevoUmbral = HALL_DEBOUNCE_US;
  if (nuevoUmbral > HALL_DEBOUNCE_MAX_US) nuevoUmbral = HALL_DEBOUNCE_MAX_US;
  hallMinPeriodUs = nuevoUmbral;

  float rawRPM = HALL_CONST_RPM   / periodoMediano;
  float rawVel = HALL_CONST_SPEED / periodoMediano;

  filteredRPM_Last = (SMOOTH_ALPHA * rawRPM) + ((1.0f - SMOOTH_ALPHA) * filteredRPM_Last);
  filteredVel_Last = (SMOOTH_ALPHA * rawVel) + ((1.0f - SMOOTH_ALPHA) * filteredVel_Last);

  currentRPM           = filteredRPM_Last;
  currentLinearSpeedMs = filteredVel_Last;

  // Velocidad de display: promedio sobre la ventana de 1 vuelta.
  // vel = K / (sum/count) = K * count / sum
  if (revPeriodSum > 0) {
    displaySpeedMs = HALL_CONST_SPEED * (float)revBufCount / (float)revPeriodSum;
  }

  if (hallDebugActive && revCompleted) {
    Serial.printf("[HALL] -- vuelta completa | Vel:%.3f m/s | RPM:%.1f\n",
                  currentLinearSpeedMs, currentRPM);
  }

  // --- CSV SERIAL: una línea por imán, con vel filtrada actualizada como referencia ---
  if (csvLogActive && validCount > 0 && setpointSpeed > 0.001f) {
    unsigned long t = millis();
    for (uint8_t i = 0; i < validCount; i++) {
      float velCruda = HALL_CONST_SPEED / (float)rawPeriods[i];
      float errRel   = (velCruda - setpointSpeed) / setpointSpeed;
      Serial.printf("%lu,%lu,%.4f,%.4f,%.4f\n",
        t, (unsigned long)rawPeriods[i],
        velCruda, setpointSpeed, errRel);
    }
  }

  // --- EJECUTAR PID MANUAL (event-driven) CADA VEZ QUE HAY UN PULSO ---
  computePID();
}

// ============================================================
// 17. LECTURA DE CORRIENTE
// ============================================================
// updateCurrentReading: promedia 20 lecturas del sensor de corriente (cada CURRENT_READ_INTERVAL)
// y las convierte a amperes usando el offset y la sensibilidad del sensor
static void updateCurrentReading() {
  if (millis() - lastCurrentReadTime < CURRENT_READ_INTERVAL) return;
  lastCurrentReadTime = millis();

  uint32_t sum = 0;
  for (int i = 0; i < 20; i++) sum += analogRead(MOTOR_CS_PIN);

  float voltage   = (sum / 20.0f) * (3.3f / 4095.0f);
  float corrected = max(voltage - CS_OFFSET_VOLTAGE, 0.0f);
  cachedCurrent   = corrected / CS_VOLTAGE_PER_AMP;
}

// ============================================================
// 18. COMANDOS SERIAL
// ============================================================
// procesarComandoSerial: interpreta un comando de texto recibido por Serial
// (SET SP, GET VEL/ENC/CUR, STOP, PWM, TUNE, HALL ON/OFF, LOG START/STOP, HELP)
void procesarComandoSerial(const char* cmd) {
  char buf[64];
  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  for (char* p = buf; *p; p++) *p = toupper((unsigned char)*p);

  float v1;
  int pwmVal;

  if (strncmp(buf, "SET SP ", 7) == 0 && sscanf(buf + 7, "%f", &v1) == 1) {
    float sp = constrain(v1, 0.0f, 2.0f);
    setpointSpeed   = sp;
    pendingSetpoint = sp;
    setpointPending = false;

    if (sp == 0.0f) {
      motorStop();
      Serial.println("[SP] Motor detenido");
    } else {
      configurarMotorReversa();
      activarPID(sp);
      Serial.printf("[SP] Consigna: %.3f m/s\n", sp);
    }

  } else if (strcmp(buf, "GET VEL") == 0) {
    float error = setpointSpeed - currentLinearSpeedMs;
    Serial.printf("[VEL] %.3f m/s | SP: %.3f | PWM: %.0f | FF: %.0f | Int: %.3f | Err: %.3f | Ki*Int: %.1f\n",
      currentLinearSpeedMs, setpointSpeed, pidOutput, lastFF_PWM, 
      pidIntegral, error, Ki * pidIntegral);

  } else if (strcmp(buf, "GET ENC") == 0) {
    Serial.printf("[ENC] Raw: %d | Display: %d\n",
      encoder.getCount(), displayEncoderCount);

  } else if (strcmp(buf, "RESET ENC") == 0) {
    encoder.clearCount();
    lastStableCount     = 0;
    displayEncoderCount = 0;
    Serial.println("[ENC] Contadores reseteados");

  } else if (strcmp(buf, "GET CUR") == 0) {
    Serial.printf("[CUR] %.2f A\n", cachedCurrent);

  } else if (strcmp(buf, "STOP") == 0) {
    motorStop();
    Serial.println("[STOP] Motor detenido");

  } else if ( (strncmp(buf, "PWM ", 4) == 0 && sscanf(buf + 4, "%d", &pwmVal) == 1)
           || (strncmp(buf, "SET PWM ", 8) == 0 && sscanf(buf + 8, "%d", &pwmVal) == 1) ) {
    int pwm = (int)constrain(pwmVal, 0, MAX_VELOCITY);
    // Limpiar setpoint para que computePID() no sobreescriba el PWM manual
    setpointSpeed   = 0.0f;
    pendingSetpoint = 0.0f;
    setpointPending = false;
    myPID.SetMode(MANUAL);
    pidOut      = (double)pwm;
    pidIntegral = 0.0f;
    pidOutput   = (float)pwm;
    if (pwm > 0) configurarMotorReversa();
    else         motorParadaLibre();
    applyPWM(pwm);
    Serial.printf("[PWM] Manual PWM aplicado: %d\n", pwm);

  } else if (strncmp(buf, "TUNE ", 5) == 0) {
    float p, i, d;
    if (sscanf(buf + 5, "%f %f %f", &p, &i, &d) == 3) {
      Kp = p; Ki = i; Kd = d;
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.printf("[TUNE] Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
    }

  } else if (strcmp(buf, "HALL ON") == 0) {
    hallDebugActive = true;
    magnetCount     = 0;
    Serial.println("[HALL] Monitor activado — imprime cada iman y cada vuelta");

  } else if (strcmp(buf, "HALL OFF") == 0) {
    hallDebugActive = false;
    Serial.println("[HALL] Monitor desactivado");

  } else if (strcmp(buf, "LOG START") == 0) {
    csvLogActive = true;
    Serial.println("t_ms,periodo_us,vel_cruda_m_s,setpoint_m_s,error_rel");

  } else if (strcmp(buf, "LOG STOP") == 0) {
    csvLogActive = false;
    Serial.println("[LOG] Detenido");

  } else if (strcmp(buf, "HELP") == 0) {
    Serial.println("CMD: SET SP <m/s> | GET VEL | GET ENC | RESET ENC | GET CUR | STOP | TUNE <Kp Ki Kd> | PWM <0..MAX> | LOG START | LOG STOP | HALL ON | HALL OFF");

  } else {
    Serial.printf("[ERR] Desconocido: '%s'\n", cmd);
  }
}

// leerSerial: arma línea a línea los bytes entrantes de Serial y despacha cada
// comando completo (terminado en \n o \r) a procesarComandoSerial()
static void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialIdx > 0) {
        serialBuf[serialIdx] = '\0';
        procesarComandoSerial(serialBuf);
        serialIdx = 0;
      }
    } else if (serialIdx < (sizeof(serialBuf) - 1)) {
      serialBuf[serialIdx++] = c;
    }
  }
}

// ============================================================
// 19. WEBSOCKET
// ============================================================
// broadcastState: arma el JSON de estado (velocidad, setpoint, PID, encoder, corriente,
// log crudo de pulsos) y lo envía por WebSocket a todos los clientes conectados
void broadcastState() {
  if (ws.count() == 0) {
    pulseLogCount    = 0;
    pulseLogOverflow = false;
    return;
  }

  JsonDocument doc;
  doc["velocity"]       = (int)round(pidOutput);
  doc["current"]        = cachedCurrent;
  doc["rpm"]            = currentRPM;
  doc["speed_m_s"]      = currentLinearSpeedMs;
  doc["speed_mm_s"]     = currentLinearSpeedMs * 1000.0f;
  doc["sp_m_s"]         = setpointSpeed;
  doc["sp_pending_m_s"] = setpointPending ? pendingSetpoint : setpointSpeed;
  doc["sp_pending"]     = setpointPending;
  doc["encoder"]        = displayEncoderCount;
  doc["encoder_raw"]    = encoder.getCount();
  doc["pid_integral"]   = pidIntegral;
  doc["ff_pwm"]         = lastFF_PWM;
  doc["kp"]             = Kp;
  doc["ki"]             = Ki;
  doc["kd"]             = Kd;

  // Log crudo imán por imán
  if (pulseLogCount > 0) {
    JsonArray pulses = doc["pulses"].to<JsonArray>();
    for (uint8_t i = 0; i < pulseLogCount; i++) {
      JsonObject p = pulses.add<JsonObject>();
      p["p"] = pulseLogBuffer[i].periodo_us;
      p["v"] = pulseLogBuffer[i].vel_m_s;
      p["r"] = pulseLogBuffer[i].rpm;
    }
    pulseLogCount = 0;
  }
  if (pulseLogOverflow) {
    doc["pulses_overflow"] = true;
    pulseLogOverflow = false;
  }

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// onWebSocketEvent: maneja conexión/desconexión de clientes y los mensajes JSON
// entrantes (led, sp, sp_pending, confirm_sp, stop, tune) desde la interfaz web
void onWebSocketEvent(AsyncWebSocket* srv, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (ws.count() > 2) { client->close(); return; }
    Serial.printf("[WS] Cliente conectado (id=%u, total=%u)\n",
                  client->id(), ws.count());
    isWebConnected = true;
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    broadcastState();

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente desconectado (id=%u, total=%u)\n",
                  client->id(), ws.count());
    if (ws.count() == 0) {
      isWebConnected = false;
      digitalWrite(LED_BUILTIN_PIN, LOW);
    }

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (!info->final || info->index != 0 || info->len != len
        || info->opcode != WS_TEXT) return;

    JsonDocument doc;
    if (deserializeJson(doc, data, len)) return;

    bool changed = false;

    if (doc["led"].is<bool>()) {
      digitalWrite(LED_BUILTIN_PIN, doc["led"].as<bool>() ? HIGH : LOW);
      changed = true;
    }

    if (doc["sp"].is<float>() || doc["sp"].is<int>()) {
      float sp = constrain(doc["sp"].as<float>(), 0.0f, 2.0f);
      setpointSpeed   = sp;
      pendingSetpoint = sp;
      setpointPending = false;

      if (sp == 0.0f) {
        motorStop();
      } else {
        configurarMotorReversa();
        activarPID(sp);
      }
      changed = true;
    }

    if (doc["sp_pending"].is<float>() || doc["sp_pending"].is<int>()) {
      pendingSetpoint = constrain(doc["sp_pending"].as<float>(), 0.0f, 2.0f);
      setpointPending = true;
      changed = true;
    }

    if (doc["confirm_sp"].is<bool>() && doc["confirm_sp"].as<bool>()) {
      confirmarSetpoint();
      changed = true;
    }

    if (doc["stop"].is<bool>() && doc["stop"].as<bool>()) {
      motorStop();
      changed = true;
    }

    if (doc["tune"].is<JsonObject>()) {
      JsonObject t = doc["tune"].as<JsonObject>();
      Kp = t["kp"] | 50.0;
      Ki = t["ki"] | 5.0;
      Kd = t["kd"] | 0.0;
      myPID.SetTunings(Kp, Ki, Kd);
      Serial.printf("[TUNE WS] Kp=%.2f Ki=%.2f Kd=%.2f\n", Kp, Ki, Kd);
      changed = true;
    }

    if (changed) broadcastState();
  }
}

// ============================================================
// 20. PÁGINA DE DIAGNÓSTICO (FALLBACK)
// ============================================================
static const char FALLBACK_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="es"><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta charset="UTF-8">
<title>Trotadora PID v9.11.0</title>
<style>
body{font-family:monospace;background:#0d0f14;color:#c8d0e0;padding:30px;max-width:620px;margin:0 auto}
h2{color:#00e5ff}
.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #1e2330}
button{padding:10px;background:#1e2330;border-radius:4px;cursor:pointer;margin:5px}
.row{display:flex;gap:8px;flex-wrap:wrap}
.pending{color:#ffaa00}
.confirmed{color:#39ff6e}
.ff{color:#ff8844}
.highlight{color:#00e5ff}
</style></head><body>
<h2>Trotadora PID v9.11.0</h2>
<div id="status">Conectando WebSocket...</div>
<div id="metrics"></div>
<div class="row">
  <button onclick="send({sp:0.5})">SP 0.5 m/s</button>
  <button onclick="send({sp:1.0})">SP 1.0 m/s</button>
  <button onclick="send({sp:1.5})">SP 1.5 m/s</button>
  <button onclick="send({stop:true})">STOP</button>
</div>
<div class="row">
  <input type="range" id="spSlider" min="0" max="200" step="1" value="0">
  <span id="sliderValue">0.00 m/s</span>
  <button onclick="send({sp_pending: parseFloat(document.getElementById('sliderValue').innerText)})">Set Pending</button>
  <button onclick="send({confirm_sp: true})">CONFIRMAR</button>
</div>
<script>
var ws;
function init(){
  ws=new WebSocket('ws://'+location.hostname+'/ws');
  ws.onopen=function(){document.getElementById('status').innerHTML='<span style="color:#39ff6e">Conectado</span>'};
  ws.onmessage=function(e){
    var d=JSON.parse(e.data);
    var pendingHtml = d.sp_pending ?
      '<span class="pending">PENDIENTE: '+d.sp_pending_m_s.toFixed(3)+' m/s</span>' :
      '<span class="confirmed">CONFIRMADO: '+d.sp_m_s.toFixed(3)+' m/s</span>';
    document.getElementById('metrics').innerHTML=`
      <div class="metric"><span>Velocidad actual</span><span>${(d.speed_m_s||0).toFixed(3)} m/s</span></div>
      <div class="metric"><span>Setpoint</span><span>${pendingHtml}</span></div>
      <div class="metric"><span>PWM</span><span>${d.velocity||0}</span></div>
      <div class="metric"><span>FF PWM</span><span class="ff">${(d.ff_pwm||0).toFixed(0)}</span></div>
      <div class="metric"><span>Ki × Integral</span><span class="highlight">${((d.ki||0) * (d.pid_integral||0)).toFixed(0)}</span></div>
      <div class="metric"><span>Encoder</span><span>${d.encoder||0}</span></div>
      <div class="metric"><span>Corriente</span><span>${(d.current||0).toFixed(2)} A</span></div>
      <div class="metric"><span>Kp=${(d.kp||0).toFixed(1)}</span><span>Ki=${(d.ki||0).toFixed(1)} Kd=${(d.kd||0).toFixed(1)}</span></div>
    `;
    document.getElementById('spSlider').value = d.sp_pending_m_s * 100;
    document.getElementById('sliderValue').innerText = d.sp_pending_m_s.toFixed(2);
  };
  ws.onclose=function(){document.getElementById('status').innerHTML='<span style="color:#ff3355">Desconectado</span>';setTimeout(init,2000)};
}
function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}
var slider=document.getElementById('spSlider');
slider.oninput=function(){
  var val=this.value/100;
  document.getElementById('sliderValue').innerText=val.toFixed(2);
};
window.onload=init;
</script></body></html>
)HTML";

// ============================================================
// 21. HTTP
// ============================================================
// setupHTTPEndpoints: sirve la SPA desde LittleFS ("/") y registra una página
// de diagnóstico embebida (FALLBACK_PAGE) para cuando el filesystem no está disponible
void setupHTTPEndpoints() {
  if (LittleFS.begin(true)) {
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    Serial.println("[FS] LittleFS OK");
  } else {
    Serial.println("[FS] LittleFS no disponible — usando fallback");
  }
  server.onNotFound([](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", FALLBACK_PAGE);
  });
}

// ============================================================
// 22. LCD
// ============================================================
// scanI2CAddress: barre el bus I2C buscando una dirección conocida de LCD (0x27 o 0x3F)
uint8_t scanI2CAddress() {
  for (uint8_t addr = 0x20; addr < 0x40; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (addr == 0x27 || addr == 0x3F) return addr;
    }
  }
  return 0;
}

// initLCD: inicializa el bus I2C, detecta la dirección del LCD y lo deja listo para usar
void initLCD() {
  if (lcd) return;
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  Wire.setClock(400000);
  uint8_t addr = scanI2CAddress();
  if (!addr) { Serial.println("[LCD] No I2C device"); return; }
  lcd = new LiquidCrystal_I2C(addr, 16, 2);
  lcd->init();
  lcd->backlight();
  lcd->clear();
  Serial.printf("[LCD] OK @ 0x%02X\n", addr);
}

// updateLCD: refresca la pantalla (cada LCD_UPDATE_INTERVAL) con setpoint,
// velocidad, PWM y corriente actuales
void updateLCD() {
  if (!lcd) return;
  if (millis() - lastLcdUpdate < LCD_UPDATE_INTERVAL) return;
  lastLcdUpdate = millis();

  char row0[17];
  char row1[17];

  if (setpointPending) {
    snprintf(row0, sizeof(row0), "SP:%-4.2f*->%-4.2f", setpointSpeed, pendingSetpoint);
  } else {
    snprintf(row0, sizeof(row0), "SP:%-4.2f V:%-4.2f", setpointSpeed, displaySpeedMs);
  }
  lcd->setCursor(1, 0);
  lcd->print(row0);

  snprintf(row1, sizeof(row1), "PWM:%-4d I:%-4.2fA", (int)round(pidOutput), cachedCurrent);
  lcd->setCursor(1, 1);
  lcd->print(row1);
}

// ============================================================
// 23. DEBUG SERIAL
// ============================================================
// debugSerial: imprime por Serial un resumen periódico (cada DEBUG_INTERVAL) del
// estado de velocidad/PID/corriente, salvo que el log CSV esté activo
void debugSerial() {
  if (csvLogActive) return;
  if (millis() - lastDebugTime < DEBUG_INTERVAL) return;
  lastDebugTime = millis();
  float error = setpointSpeed - currentLinearSpeedMs;
  Serial.printf("[STATS] Vel:%.3f | SP:%.3f | PWM:%.0f | FF:%.0f | Int:%.3f | Ki*Int:%.1f | Err:%.3f | Cur:%.2fA\n",
    currentLinearSpeedMs, setpointSpeed, pidOutput, lastFF_PWM,
    pidIntegral, Ki * pidIntegral, error, cachedCurrent);
}

// ============================================================
// 24. SETUP
// ============================================================
// setup: configura pines de motor/Hall/encoder/emergencia, arranca el PID en MANUAL,
// inicializa LCD, levanta el WiFi AP y el servidor HTTP/WebSocket
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Trotadora PID v9.11.0 - Anti-windup simétrico");

  // Motor
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  motorParadaLibre();

#if ESP_IDF_VERSION_MAJOR >= 5
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcWrite(MOTOR_PWM_PIN, 0);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
  ledcWrite(PWM_CHANNEL, 0);
#endif

  pinMode(MOTOR_CS_PIN,   INPUT);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);

  // Hall
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), ISR_SensorHall, FALLING);

  // Encoder
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  encoder.attachHalfQuad(ENCODER_DT_PIN, ENCODER_CLK_PIN);
  encoder.clearCount();
  encoder.setFilter(1023);

  lastStableCount     = 0;
  lastStableTime      = millis();
  displayEncoderCount = 0;
  pendingSetpoint     = 0.0f;
  setpointPending     = false;

  Serial.println("[ENCODER] Inicializado — HALF_QUAD");

  // Botón del encoder
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), ISR_EncoderBoton, FALLING);

  // Emergencia
  pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), ISR_EmergencyStop, FALLING);

  // PID Brett Beauregard — arranca en reposo
  myPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  myPID.SetSampleTime(PID_TIMER_INTERVAL_MS);
  myPID.SetMode(MANUAL);  // permanece en MANUAL hasta activarPID()
  pidOut            = 0.0;
  pidOutput         = 0.0f;
  pidIntegral       = 0.0f;
  lastControlMicros = micros();

  Serial.printf("[PID] Brett Beauregard | Kp=%.2f Ki=%.2f Kd=%.2f | PWM:[%d..%d] | Sample:%lums\n",
                Kp, Ki, Kd, PID_OUTPUT_MIN, PID_OUTPUT_MAX, PID_TIMER_INTERVAL_MS);
  Serial.println("[PID] Esperando setpoint...");

  // LCD
  initLCD();
  if (lcd) {
    lcd->setCursor(1, 0); lcd->print("TROTADORA v9.11.0");
    lcd->setCursor(1, 1); lcd->print("ANTI-WINDUP");
    delay(1500);
    lcd->clear();
  }

  // WiFi AP
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, apPassword);
  Serial.printf("[WiFi] AP: %s | IP: %s\n",
                ssid, WiFi.softAPIP().toString().c_str());

  // WebSocket + HTTP
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  setupHTTPEndpoints();
  server.begin();
  Serial.println("[HTTP] Servidor iniciado");

  Serial.printf("[INFO] Rodillo Ø%.1fmm | %d imanes | PWM_MIN:%d PWM_MAX:%d\n",
                DRIVE_ROLLER_DIAMETER_MM, MAGNETS_COUNT,
                PID_OUTPUT_MIN, PID_OUTPUT_MAX);
  Serial.println("[INFO] Comandos: SET SP <m/s> | GET VEL | GET ENC | RESET ENC | GET CUR | STOP | TUNE <Kp Ki Kd>");
  Serial.println("[INFO] Encoder: Gira para ajustar setpoint temporal, presiona para CONFIRMAR");
}

// ============================================================
// 25. LOOP PRINCIPAL
// ============================================================
// loop: procesa parada de emergencia, lee encoder/serial, calcula velocidad y
// PID event-driven, con timer fallback para garantizar cadencia mínima del PID;
// además lee corriente, transmite estado por WebSocket y refresca LCD/debug
void loop() {
  unsigned long ahora = millis();

  // Emergencia
  if (emergencyStopTriggered) {
    if (!emergencyStopPending) {
      motorStop();
      Serial.println("[EMERGENCIA] ¡Motor detenido!");
      broadcastState();
      emergencyStopPending  = true;
      lastEmergencyDebounce = ahora;
    } else if (ahora - lastEmergencyDebounce >= EMERGENCY_DEBOUNCE_MS) {
      emergencyStopTriggered = false;
      emergencyStopPending   = false;
    }
  }

  // Entradas
  procesarEntradasEncoder();
  leerSerial();

  // Cálculo de velocidad + PID event-driven (se ejecuta en calculateSpeed)
  calculateSpeed();

  // Timer fallback: garantizar mínimo 20 Hz al PID aunque los pulsos Hall sean escasos
  // (ej: cinta frenada con la mano → menos pulsos → PID reaccionaba más lento)
  if (setpointSpeed > 0.0f &&
      (micros() - lastControlMicros) >= (PID_TIMER_INTERVAL_MS * 1000UL)) {
    computePID();
  }

  // Lectura de corriente
  updateCurrentReading();

  // WebSocket
  ws.cleanupClients();
  if (!isWebConnected) {
    if (ahora - lastBlink > 500) {
      lastBlink = ahora;
      digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
    }
  } else if (ahora - lastWsSendTime > WS_SEND_INTERVAL) {
    lastWsSendTime = ahora;
    broadcastState();
  }

  updateLCD();
  debugSerial();

  delay(1);
}