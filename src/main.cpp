// ============================================================
//  TROTADORA PARA RATAS — Control PID v9.7
//  Plataforma: ESP32 + Driver VNH5019 + Sensor Hall + KY-040
//
//  Cambios v9.6 → v9.7:
//    - PID arranca en modo MANUAL, solo se activa con setpoint > 0
//    - Límites PID: PID_OUTPUT_MIN (150) a MAX_VELOCITY al activar
//    - Feed-forward al confirmar setpoint para respuesta rápida
//    - motorStop() restaura límites neutros y pone PID en manual
//    - Eliminado arranqueEnProceso (ya no es necesario)
// ============================================================

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QuickPID.h>
#include <algorithm>
#include "motor_config.h"
#include <ESP32Encoder.h>

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
#undef  HALL_DEBOUNCE_US
#define HALL_DEBOUNCE_US  1000UL

// Límites de salida del PID
#define PID_OUTPUT_MIN   150          // PWM mínimo para vencer fricción estática
#define PID_OUTPUT_MAX   MAX_VELOCITY // 1023

static_assert(HALL_BUFFER_SIZE > 1, "HALL_BUFFER_SIZE debe ser > 1");

static const float HALL_CONST_RPM   = 60000000.0f / (float)MAGNETS_COUNT;
static const float HALL_CONST_SPEED = (3.14159265f * DRIVE_ROLLER_DIAMETER_MM * 1000.0f)
                                       / (float)MAGNETS_COUNT;

// ============================================================
// 3. TEMPORIZACIÓN
// ============================================================
#define CONTROL_INTERVAL_MS   100UL
#define LCD_UPDATE_INTERVAL   300UL
#define DEBUG_INTERVAL        1000UL
#define CURRENT_READ_INTERVAL 50UL

// ============================================================
// 4. FILTRO DE MEDIANA + EMA
// ============================================================
#define FILTER_SIZE   5
#define SMOOTH_ALPHA  0.5f

static uint32_t filterBuffer[FILTER_SIZE];
static uint8_t  filterIndex = 0;
static uint8_t  filterCount = 0;

static float filteredRPM_Last = 0.0f;
static float filteredVel_Last = 0.0f;

// ============================================================
// 5. VARIABLES DE VELOCIDAD / PID
// ============================================================
float currentRPM           = 0.0f;
float currentLinearSpeedMs = 0.0f;
float setpointSpeed        = 0.0f;   // Setpoint activo (confirmado)
float pendingSetpoint      = 0.0f;   // Setpoint temporal del encoder
float pidOutput            = 0.0f;
bool  setpointPending      = false;  // Hay un setpoint pendiente de confirmar

QuickPID myPID(&currentLinearSpeedMs, &pidOutput, &setpointSpeed,
               30.0f, 10.0f, 4.0f, QuickPID::Action::direct);

// ============================================================
// 6. BUFFER CIRCULAR HALL
// ============================================================
static volatile uint32_t hallPeriodBuffer[HALL_BUFFER_SIZE] = {0};
static volatile uint8_t  hallBufferHead  = 0;
static volatile uint8_t  hallBufferCount = 0;
static volatile uint32_t lastHallMicros  = 0;
static uint8_t           lastProcessedHead = 0;

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

// ============================================================
// 14. IMPLEMENTACIÓN DE FUNCIONES
// ============================================================

void applyPWM(int pwmVal) {
  pwmVal = constrain(pwmVal, 0, MAX_VELOCITY);
#if ESP_IDF_VERSION_MAJOR >= 5
  ledcWrite(MOTOR_PWM_PIN, (uint32_t)pwmVal);
#else
  ledcWrite(PWM_CHANNEL, (uint32_t)pwmVal);
#endif
}

void configurarMotorReversa() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, HIGH);
}

void motorParadaLibre() {
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

// ---------------------
// motorStop: apaga motor, limites neutros, PID en manual
// ---------------------
void motorStop() {
  setpointSpeed   = 0.0f;
  pendingSetpoint = 0.0f;
  setpointPending = false;
  pidOutput       = 0.0f;
  applyPWM(0);
  motorParadaLibre();

  // Límites neutros y PID en manual para que no actúe sin setpoint
  myPID.SetOutputLimits(0, PID_OUTPUT_MAX);
  myPID.SetMode(QuickPID::Control::manual);
}

// ---------------------
// activarPID: feed-forward + límites correctos + modo automático
// ---------------------
static void activarPID(float sp) {
  // Reset integrador: manual → automático
  myPID.SetMode(QuickPID::Control::manual);

  // Feed-forward proporcional al setpoint (precarga pidOutput)
  pidOutput = PID_OUTPUT_MIN
              + (sp / 2.0f) * (float)(PID_OUTPUT_MAX - PID_OUTPUT_MIN);
  pidOutput = constrain(pidOutput, (float)PID_OUTPUT_MIN, (float)PID_OUTPUT_MAX);

  // Límites reales: nunca baja de PID_OUTPUT_MIN mientras hay setpoint
  myPID.SetOutputLimits(PID_OUTPUT_MIN, PID_OUTPUT_MAX);

  // Activar PID en automático con pidOutput precargado
  myPID.SetMode(QuickPID::Control::automatic);

  applyPWM((int)pidOutput);
  Serial.printf("[PID] Activado — SP: %.3f m/s | FF PWM: %.0f\n", sp, pidOutput);
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
void IRAM_ATTR ISR_SensorHall() {
  uint32_t ahora = micros();

  if (lastHallMicros == 0) {
    lastHallMicros = ahora;
    return;
  }

  uint32_t periodo = ahora - lastHallMicros;
  lastHallMicros = ahora;

  if (periodo >= HALL_DEBOUNCE_US) {
    hallPeriodBuffer[hallBufferHead] = periodo;
    hallBufferHead = (hallBufferHead + 1) % HALL_BUFFER_SIZE;
    if (hallBufferCount < HALL_BUFFER_SIZE) hallBufferCount++;
  }
}

void IRAM_ATTR ISR_EncoderBoton() {
  if (millis() - lastButtonDebounce > ENCODER_BUTTON_DEBOUNCE_MS) {
    encoderButtonPressed = true;
  }
}

void IRAM_ATTR ISR_EmergencyStop() {
  emergencyStopTriggered = true;
}

// ============================================================
// 16. CÁLCULO DE VELOCIDAD
// ============================================================
static float getMedian(uint32_t* arr, uint8_t size, uint8_t count) {
  uint32_t temp[FILTER_SIZE];
  uint8_t  n = (count < size) ? count : size;
  if (n == 0) return 0.0f;
  memcpy(temp, arr, n * sizeof(uint32_t));
  std::sort(temp, temp + n);
  return (float)temp[n / 2];
}

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

    portDISABLE_INTERRUPTS();
    hallBufferCount   = 0;
    hallBufferHead    = 0;
    lastProcessedHead = 0;
    memset((void*)hallPeriodBuffer, 0, sizeof(hallPeriodBuffer));
    portENABLE_INTERRUPTS();
    return;
  }

  uint8_t prevProcessedHead = lastProcessedHead;
  uint8_t newPulses = (currentHead - prevProcessedHead + HALL_BUFFER_SIZE) % HALL_BUFFER_SIZE;
  lastProcessedHead = currentHead;

  if (newPulses == 0) return;

  for (uint8_t i = 0; i < newPulses; i++) {
    uint8_t  idx = (prevProcessedHead + i) % HALL_BUFFER_SIZE;
    uint32_t p   = periods[idx];
    if (p > 200 && p < HALL_TIMEOUT_US) {
      filterBuffer[filterIndex] = p;
      filterIndex = (filterIndex + 1) % FILTER_SIZE;
      if (filterCount < FILTER_SIZE) filterCount++;
    }
  }

  if (filterCount == 0) return;

  float periodoMediano = getMedian(filterBuffer, FILTER_SIZE, filterCount);

  float rawRPM = HALL_CONST_RPM   / periodoMediano;
  float rawVel = HALL_CONST_SPEED / periodoMediano;

  filteredRPM_Last = (SMOOTH_ALPHA * rawRPM) + ((1.0f - SMOOTH_ALPHA) * filteredRPM_Last);
  filteredVel_Last = (SMOOTH_ALPHA * rawVel) + ((1.0f - SMOOTH_ALPHA) * filteredVel_Last);

  currentRPM           = filteredRPM_Last;
  currentLinearSpeedMs = filteredVel_Last;
}

// ============================================================
// 17. LECTURA DE CORRIENTE
// ============================================================
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
void procesarComandoSerial(const char* cmd) {
  char buf[64];
  strncpy(buf, cmd, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';
  for (char* p = buf; *p; p++) *p = toupper((unsigned char)*p);

  float v1;

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
    Serial.printf("[VEL] %.3f m/s | SP: %.3f | PWM: %.0f | Pending: %s\n",
      currentLinearSpeedMs, setpointSpeed, pidOutput,
      setpointPending ? "YES" : "NO");

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

  } else if (strncmp(buf, "TUNE ", 5) == 0) {
    float p, i, d;
    if (sscanf(buf + 5, "%f %f %f", &p, &i, &d) == 3) {
      myPID.SetTunings(p, i, d);
      Serial.printf("[TUNE] Kp=%.2f Ki=%.2f Kd=%.2f\n", p, i, d);
    }

  } else if (strcmp(buf, "HELP") == 0) {
    Serial.println("CMD: SET SP <m/s> | GET VEL | GET ENC | RESET ENC | GET CUR | STOP | TUNE <Kp Ki Kd>");

  } else {
    Serial.printf("[ERR] Desconocido: '%s'\n", cmd);
  }
}

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
void broadcastState() {
  if (ws.count() == 0) return;

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

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

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
      float kp = t["kp"] | 30.0f;
      float ki = t["ki"] | 10.0f;
      float kd = t["kd"] |  4.0f;
      myPID.SetTunings(kp, ki, kd);
      Serial.printf("[TUNE WS] Kp=%.2f Ki=%.2f Kd=%.2f\n", kp, ki, kd);
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
<title>Trotadora PID v9.7</title>
<style>
body{font-family:monospace;background:#0d0f14;color:#c8d0e0;padding:30px;max-width:620px;margin:0 auto}
h2{color:#00e5ff}
.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #1e2330}
button{padding:10px;background:#1e2330;border-radius:4px;cursor:pointer;margin:5px}
.row{display:flex;gap:8px;flex-wrap:wrap}
.pending{color:#ffaa00}
.confirmed{color:#39ff6e}
</style></head><body>
<h2>Trotadora PID v9.7</h2>
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
      <div class="metric"><span>Encoder</span><span>${d.encoder||0}</span></div>
      <div class="metric"><span>Raw Encoder</span><span>${d.encoder_raw||0}</span></div>
      <div class="metric"><span>Corriente</span><span>${(d.current||0).toFixed(2)} A</span></div>
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
uint8_t scanI2CAddress() {
  for (uint8_t addr = 0x20; addr < 0x40; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (addr == 0x27 || addr == 0x3F) return addr;
    }
  }
  return 0;
}

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

void updateLCD() {
  if (!lcd) return;
  if (millis() - lastLcdUpdate < LCD_UPDATE_INTERVAL) return;
  lastLcdUpdate = millis();

  char row0[17];
  char row1[17];

  if (setpointPending) {
    snprintf(row0, sizeof(row0), "SP:%-4.2f*->%-4.2f", setpointSpeed, pendingSetpoint);
  } else {
    snprintf(row0, sizeof(row0), "SP:%-4.2f V:%-4.2f", setpointSpeed, currentLinearSpeedMs);
  }
  lcd->setCursor(0, 0);
  lcd->print(row0);

  snprintf(row1, sizeof(row1), "PWM:%-4d I:%-4.2fA", (int)round(pidOutput), cachedCurrent);
  lcd->setCursor(0, 1);
  lcd->print(row1);
}

// ============================================================
// 23. DEBUG SERIAL
// ============================================================
void debugSerial() {
  if (millis() - lastDebugTime < DEBUG_INTERVAL) return;
  lastDebugTime = millis();
  Serial.printf("[STATS] Vel:%.3f m/s SP:%.3f PWM:%.0f Cur:%.2fA Pending:%s PID:%s\n",
    currentLinearSpeedMs, setpointSpeed, pidOutput, cachedCurrent,
    setpointPending ? "YES" : "NO",
    (myPID.GetMode() == (uint8_t)QuickPID::Control::automatic) ? "AUTO" : "MANUAL");
}

// ============================================================
// 24. SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[BOOT] Trotadora PID v9.7");

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

  // PID — arranca en MANUAL, límites neutros
  // El motor NO recibirá PWM hasta que se confirme un setpoint > 0
  myPID.SetOutputLimits(0, PID_OUTPUT_MAX);
  myPID.SetMode(QuickPID::Control::manual);
  pidOutput = 0.0f;
  Serial.println("[PID] Modo MANUAL al inicio — esperando setpoint");

  // LCD
  initLCD();
  if (lcd) {
    lcd->setCursor(0, 0); lcd->print("TROTADORA v9.7");
    lcd->setCursor(0, 1); lcd->print("PID listo");
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

  // Control PID
  static unsigned long ultimoControl = 0;
  if (ahora - ultimoControl >= CONTROL_INTERVAL_MS) {
    ultimoControl = ahora;
    calculateSpeed();

    // Compute() no hace nada en modo manual; en automático regula la velocidad
    myPID.Compute();
    applyPWM((int)round(pidOutput));
  }

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

  delay(5);
}
