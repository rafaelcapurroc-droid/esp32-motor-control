#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "motor_config.h"

const char* ssid      = "ESP32_Motor_Control";
const char* apPassword = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- ESTADO DEL MOTOR ---
volatile int motorVelocity  = 0;
volatile int motorDirection = 1;
bool isWebConnected = false;
unsigned long lastWsSendTime = 0;
const unsigned long WS_SEND_INTERVAL = 200;

// Corriente
float cachedCurrent = 0.0f;
unsigned long lastCurrentReadTime = 0;
const unsigned long CURRENT_READ_INTERVAL = 50;

// Debug serial
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000;

#define LED_BUILTIN_PIN 2
unsigned long lastBlink = 0;

// --- VARIABLES SENSOR HALL (buffer circular de períodos) ---
volatile uint32_t hallPeriodBuffer[HALL_BUFFER_SIZE] = {0};
volatile uint8_t  hallBufferIndex = 0;
volatile uint8_t  hallBufferCount = 0;
volatile uint32_t lastHallMicros  = 0;

// Constantes de conversión (período en µs → m/s y RPM)
// RPM   = 60_000_000 / (MAGNETS_COUNT * periodoPromedio_µs)
// m/s   = PI * D_mm * 1000 / (MAGNETS_COUNT * periodoPromedio_µs)
static const float HALL_CONST_RPM   = 60000000.0f / MAGNETS_COUNT;
static const float HALL_CONST_SPEED = (3.14159265f * DRIVE_ROLLER_DIAMETER_MM * 1000.0f) / MAGNETS_COUNT;

float currentRPM           = 0.0f;
float currentLinearSpeedMs = 0.0f;

// --- VARIABLES ENCODER KY-040 ---
volatile int encoderPosition = 0;
unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_DEBOUNCE_MS = 5;

// --- BOTON ENCODER ---
volatile bool encoderButtonPressed = false;
unsigned long lastButtonDebounce = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// --- EMERGENCY STOP BUTTON (PIN 34) ---
// Nota: El pin 34 requiere resistencia pull-up externa de 10kΩ a 3.3V
volatile bool emergencyStopActive = false;
unsigned long lastEmergencyDebounce = 0;
const unsigned long EMERGENCY_DEBOUNCE_MS = 100;

// --- LCD 16x2 I2C ---
LiquidCrystal_I2C lcd(0x27, 16, 2);
int pendingVelocity = 0;
unsigned long lastLcdUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 200;

// ============================================================
// ISRs
// ============================================================
void IRAM_ATTR onHallSensorTrigger() {
  uint32_t now = micros();
  uint32_t elapsed = now - lastHallMicros;

  if (elapsed > HALL_DEBOUNCE_US) {
    hallPeriodBuffer[hallBufferIndex] = elapsed;
    hallBufferIndex = (hallBufferIndex + 1) % HALL_BUFFER_SIZE;
    if (hallBufferCount < HALL_BUFFER_SIZE) hallBufferCount++;
    lastHallMicros = now;
  }
}

void IRAM_ATTR onEncoderInterrupt() {
  unsigned long now = millis();
  if (now - lastEncoderReadTime < ENCODER_DEBOUNCE_MS) return;
  lastEncoderReadTime = now;

  int clk = digitalRead(ENCODER_CLK_PIN);
  int dt  = digitalRead(ENCODER_DT_PIN);

  if (clk == LOW) {
    if (dt == HIGH) {
      pendingVelocity += ENCODER_VELOCITY_STEP;
    } else {
      pendingVelocity -= ENCODER_VELOCITY_STEP;
    }
  }

  if (pendingVelocity > MAX_VELOCITY) pendingVelocity = MAX_VELOCITY;
  if (pendingVelocity < 0)            pendingVelocity = 0;
}

void IRAM_ATTR onEncoderButton() {
  unsigned long now = millis();
  if (now - lastButtonDebounce < BUTTON_DEBOUNCE_MS) return;
  lastButtonDebounce   = now;
  encoderButtonPressed = true;
}

void IRAM_ATTR onEmergencyStop() {
  unsigned long now = millis();
  if (now - lastEmergencyDebounce < EMERGENCY_DEBOUNCE_MS) return;
  lastEmergencyDebounce = now;
  emergencyStopActive = true;
}

// ============================================================
// Cálculo de velocidad — buffer circular de períodos
// ============================================================
void calculateSpeed() {
  static unsigned long lastCalcTime = 0;
  const unsigned long CALC_INTERVAL_MS = 500;

  if (millis() - lastCalcTime < CALC_INTERVAL_MS) return;
  lastCalcTime = millis();

  // Copia atómica del buffer (equivalente al noInterrupts() del Arduino Uno)
  uint32_t periodos[HALL_BUFFER_SIZE];
  uint8_t  count;
  uint32_t ultimoPulso;

  portDISABLE_INTERRUPTS();
  memcpy(periodos, (const void*)hallPeriodBuffer, sizeof(periodos));
  count       = hallBufferCount;
  ultimoPulso = lastHallMicros;
  portENABLE_INTERRUPTS();

  // Timeout: 2 segundos sin pulsos → velocidad cero
  bool timeout = ((micros() - ultimoPulso) > HALL_TIMEOUT_US);

  if (timeout || count == 0) {
    currentRPM           = 0.0f;
    currentLinearSpeedMs = 0.0f;
    return;
  }

  // Promedio de los períodos en el buffer
  uint32_t suma = 0;
  for (uint8_t i = 0; i < HALL_BUFFER_SIZE; i++) {
    suma += periodos[i];
  }
  uint32_t periodoPromedio = suma / HALL_BUFFER_SIZE;

  // Rango válido: entre 100µs y 2,000,000µs (igual que Arduino Uno)
  if (periodoPromedio > 100 && periodoPromedio < 2000000) {
    currentLinearSpeedMs = HALL_CONST_SPEED / (float)periodoPromedio;
    currentRPM           = HALL_CONST_RPM   / (float)periodoPromedio;
  } else {
    currentRPM           = 0.0f;
    currentLinearSpeedMs = 0.0f;
  }
}

// ============================================================
// Setup motor y periféricos
// ============================================================
void setupMotor() {
#if ESP_IDF_VERSION_MAJOR >= 5
  ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
#else
  ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
#endif

  pinMode(MOTOR_IN1_PIN,    OUTPUT);
  pinMode(MOTOR_IN2_PIN,    OUTPUT);
  pinMode(MOTOR_CS_PIN,     INPUT);
  pinMode(LED_BUILTIN_PIN,  OUTPUT);

  // Hall Sensor
  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), onHallSensorTrigger, RISING);

  // Encoder
  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN,  INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN,  INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), onEncoderInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN),  onEncoderButton,    FALLING);

  // Emergency Stop Button (pin 34 - no internal pull-up)
  pinMode(EMERGENCY_STOP_PIN, INPUT);  // Pin 34 no soporta pull-up interno
  attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), onEmergencyStop, FALLING);

  // Estado inicial motor
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
#if ESP_IDF_VERSION_MAJOR >= 5
  ledcWrite(MOTOR_PWM_PIN, 0);
#else
  ledcWrite(PWM_CHANNEL, 0);
#endif
}

// ============================================================
// Loop de control del motor
// ============================================================
void updateMotor() {
  // Emergency stop check - must run before any other logic
  if (emergencyStopActive) {
    motorVelocity       = 0;
    encoderPosition     = 0;
    emergencyStopActive = false;
    // Active brake immediately
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
#if ESP_IDF_VERSION_MAJOR >= 5
    ledcWrite(MOTOR_PWM_PIN, 0);
#else
    ledcWrite(PWM_CHANNEL, 0);
#endif
    broadcastState();
    return;  // Exit without executing rest of updateMotor()
  }

  // Encoder button confirmation - apply pendingVelocity to motor
  if (encoderButtonPressed) {
    encoderButtonPressed = false;
    motorVelocity   = pendingVelocity;
    encoderPosition = pendingVelocity;
  }

  if (motorDirection > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else if (motorDirection < 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  } else {
    // Freno activo (High-High en VNH5019)
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  }

  int pwmVal = constrain(motorVelocity, 0, MAX_VELOCITY);
#if ESP_IDF_VERSION_MAJOR >= 5
  ledcWrite(MOTOR_PWM_PIN, pwmVal);
#else
  ledcWrite(PWM_CHANNEL, pwmVal);
#endif
}

// ============================================================
// Lectura de corriente
// ============================================================
void updateCurrentReading() {
  if (millis() - lastCurrentReadTime < CURRENT_READ_INTERVAL) return;
  lastCurrentReadTime = millis();

  uint32_t sum = 0;
  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MOTOR_CS_PIN);
  }

  float avgRaw          = sum / (float)samples;
  float voltage         = avgRaw * (3.3f / 4095.0f);
  float correctedVoltage = voltage - CS_OFFSET_VOLTAGE;

  if (correctedVoltage < 0.0f) correctedVoltage = 0.0f;

  cachedCurrent = correctedVoltage / CS_VOLTAGE_PER_AMP;
}

// ============================================================
// WebSocket — broadcast de estado
// ============================================================
void broadcastState() {
  if (ws.count() == 0) return;

  JsonDocument doc;
  doc["velocity"]        = motorVelocity;
  doc["pendingVelocity"] = pendingVelocity;
  doc["direction"]       = motorDirection;
  doc["ledState"]        = (digitalRead(LED_BUILTIN_PIN) == HIGH);
  doc["current"]         = cachedCurrent;
  doc["rpm"]             = currentRPM;
  doc["speed_m_s"]       = currentLinearSpeedMs;
  doc["speed_mm_s"]      = currentLinearSpeedMs * 1000.0f;

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

// ============================================================
// Debug serial
// ============================================================
void debugSerial() {
  if (millis() - lastDebugTime < DEBUG_INTERVAL) return;
  lastDebugTime = millis();

  Serial.printf("[HALL] RPM: %.1f | Belt: %.3f m/s | Current: %.2f A | PWM: %d | Encoder: %d | WS clients: %u\n",
                currentRPM, currentLinearSpeedMs, cachedCurrent,
                motorVelocity, encoderPosition, ws.count());
}

// ============================================================
// LCD Functions
// ============================================================
void lcdShowWelcome() {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("  Motor Control ");
    lcd.setCursor(0, 1);
    lcd.print("   ESP32  v1.0  ");
    delay(2500);
    lcd.clear();
}

void updateLCD() {
    if (millis() - lastLcdUpdate < LCD_UPDATE_INTERVAL) return;
    lastLcdUpdate = millis();

    // Line 0: sensed speed (RPM from Hall sensor)
    lcd.setCursor(0, 0);
    lcd.print("Sen:");
    char bufSen[12];
    snprintf(bufSen, sizeof(bufSen), "%-7.1f", currentRPM);
    lcd.print(bufSen);
    lcd.print(" RPM");

    // Line 1: desired speed (pending confirmation from encoder)
    lcd.setCursor(0, 1);
    lcd.print("Des:");
    char bufDes[12];
    snprintf(bufDes, sizeof(bufDes), "%-8d", pendingVelocity);
    lcd.print(bufDes);
    lcd.print("PWM");
}

// ============================================================
// WebSocket — handler de eventos
// ============================================================
void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (ws.count() > 2) {
      client->close();
      return;
    }
    Serial.printf("[WS] Cliente conectado (id=%u, total=%u)\n", client->id(), ws.count());
    isWebConnected = true;
    digitalWrite(LED_BUILTIN_PIN, HIGH);
    broadcastState();
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] Cliente desconectado (id=%u, total=%u)\n", client->id(), ws.count());
    if (ws.count() == 0) {
      isWebConnected = false;
      digitalWrite(LED_BUILTIN_PIN, LOW);
      Serial.println("[WS] Ultimo cliente desconectado");
    }
  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo* info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      JsonDocument doc;
      if (deserializeJson(doc, data, len)) return;

      bool stateChanged = false;
      if (doc["led"].is<bool>()) {
        digitalWrite(LED_BUILTIN_PIN, doc["led"].as<bool>() ? HIGH : LOW);
        stateChanged = true;
      }
      if (doc["velocity"].is<int>()) {
        int vel = constrain(doc["velocity"].as<int>(), 0, MAX_VELOCITY);
        if (vel != motorVelocity) {
          motorVelocity   = vel;
          encoderPosition = vel;
          pendingVelocity = vel;  // Sync pendingVelocity with WebSocket command
          stateChanged    = true;
        }
      }
      if (doc["direction"].is<int>()) {
        int newDir = (doc["direction"].as<int>() >= 0) ? 1 : -1;
        if (newDir != motorDirection) {
          motorDirection = newDir;
          stateChanged   = true;
        }
      }
      if (stateChanged) {
        broadcastState();
      }
    }
  }
}

// ============================================================
// Pagina de diagnostico embebida en flash (fallback)
// ============================================================
static const char FALLBACK_PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html><html lang="es"><head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta charset="UTF-8">
<title>ESP32 Motor - Diagnostico</title>
<style>
body{font-family:monospace;background:#0d0f14;color:#c8d0e0;padding:30px;max-width:600px;margin:0 auto}
h2{color:#00e5ff;margin-bottom:6px}
.warn{color:#ff6b35;background:#1e1500;border:1px solid #ff6b35;padding:12px;border-radius:6px;margin:16px 0}
.ok{color:#39ff6e}
code{background:#1e2330;padding:2px 6px;border-radius:3px;font-size:0.9em}
pre{background:#1e2330;padding:14px;border-radius:6px;overflow-x:auto}
.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #1e2330}
.metric:last-child{border-bottom:none}
#ws-status{color:#ff3355}
</style>
</head><body>
<h2>ESP32 Motor Control</h2>
<p>Modo diagnostico - <span id="ws-status">sin conexion WS</span></p>
<div class="warn">
<strong>Archivos no encontrados en LittleFS</strong><br>
La interfaz principal (index.html) no esta en el sistema de archivos del ESP32.<br><br>
<strong>Solucion:</strong><br>
1. Asegurate de que la carpeta <code>data/</code> existe en la raiz del proyecto<br>
2. Ejecuta: <code>pio run --target uploadfs</code>
</div>
<h3 style="color:#00e5ff;margin-top:20px">Estado del motor</h3>
<div id="metrics">
<div class="metric"><span>RPM</span><span id="m-rpm" class="ok">-</span></div>
<div class="metric"><span>Velocidad</span><span id="m-spd" class="ok">-</span></div>
<div class="metric"><span>Corriente</span><span id="m-cur" style="color:#ff6b35">-</span></div>
<div class="metric"><span>PWM</span><span id="m-pwm">-</span></div>
<div class="metric"><span>Direccion</span><span id="m-dir">-</span></div>
</div>
<h3 style="color:#00e5ff;margin-top:20px">Control basico</h3>
<label>Velocidad (0-1023):<br>
<input type="range" min="0" max="1023" value="0" id="sl" style="width:100%;margin:8px 0"
oninput="send({velocity:parseInt(this.value)})">
</label>
<div style="display:flex;gap:8px;margin-top:8px">
<button onclick="send({direction:1})"  style="flex:1;padding:10px;background:#1e2330;color:#00e5ff;border:1px solid #00e5ff;border-radius:4px;cursor:pointer">FWD</button>
<button onclick="send({direction:-1})" style="flex:1;padding:10px;background:#1e2330;color:#ff6b35;border:1px solid #ff6b35;border-radius:4px;cursor:pointer">REV</button>
<button onclick="send({velocity:0})"   style="flex:1;padding:10px;background:#1e2330;color:#ff3355;border:1px solid #ff3355;border-radius:4px;cursor:pointer">STOP</button>
</div>
<script>
var ws;
function init(){
ws=new WebSocket('ws://'+location.hostname+'/ws');
ws.onopen=function(){document.getElementById('ws-status').textContent='WebSocket OK';document.getElementById('ws-status').style.color='#39ff6e'};
ws.onmessage=function(e){
var d=JSON.parse(e.data);
if(d.rpm!==undefined)document.getElementById('m-rpm').textContent=parseFloat(d.rpm).toFixed(1)+' RPM';
if(d.speed_m_s!==undefined)document.getElementById('m-spd').textContent=parseFloat(d.speed_m_s).toFixed(3)+' m/s';
if(d.current!==undefined)document.getElementById('m-cur').textContent=parseFloat(d.current).toFixed(2)+' A';
if(d.velocity!==undefined){document.getElementById('m-pwm').textContent=d.velocity+' / 1023';document.getElementById('sl').value=d.velocity}
if(d.direction!==undefined)document.getElementById('m-dir').textContent=d.direction>=0?'FWD':'REV';
};
ws.onclose=function(){document.getElementById('ws-status').textContent='desconectado';document.getElementById('ws-status').style.color='#ff3355';setTimeout(init,2000)};
}
function send(obj){if(ws&&ws.readyState===1)ws.send(JSON.stringify(obj));}
window.onload=init;
</script>
</body></html>
)HTML";

// ============================================================
// HTTP — rutas estáticas
// ============================================================
void setupHTTPEndpoints() {
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.serveStatic("/index.html", LittleFS, "/index.html");
  server.serveStatic("/monitor.html", LittleFS, "/monitor.html");
  server.serveStatic("/js/", LittleFS, "/js/");

  server.onNotFound([](AsyncWebServerRequest* request) {
    const String path = request->url();
    if (path == "/" || path == "/index.html" || path == "/monitor.html") {
      request->send_P(200, "text/html", FALLBACK_PAGE);
      return;
    }
    request->send(404, "text/plain", "Not Found: " + path);
  });
}

// ============================================================
// Setup
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] ERROR: No se pudo inicializar LittleFS");
  } else {
    Serial.println("[FS] LittleFS OK");
    if (!LittleFS.exists("/index.html")) {
      Serial.println("[FS] AVISO: /index.html no encontrado en LittleFS");
      Serial.println("[FS] Ejecuta 'pio run --target uploadfs' para subir los archivos");
    } else {
      Serial.println("[FS] /index.html encontrado OK");
    }
    if (!LittleFS.exists("/monitor.html")) {
      Serial.println("[FS] AVISO: /monitor.html no encontrado en LittleFS");
    } else {
      Serial.println("[FS] /monitor.html encontrado OK");
    }
  }

  setupMotor();

  // LCD I2C initialization (SDA=LCD_SDA_PIN, SCL=LCD_SCL_PIN)
  Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
  lcd.init();
  lcd.backlight();
  lcdShowWelcome();
  pendingVelocity = motorVelocity;

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, apPassword);
  Serial.printf("[WiFi] AP: %s | IP: %s\n", ssid, WiFi.softAPIP().toString().c_str());

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  setupHTTPEndpoints();
  server.begin();

  Serial.println("[HTTP] Servidor iniciado");
  Serial.printf("[INFO] Rodillo Ø%.1fmm | %d imanes | CONST_SPEED=%.4f | CONST_RPM=%.0f\n",
                DRIVE_ROLLER_DIAMETER_MM, MAGNETS_COUNT, HALL_CONST_SPEED, HALL_CONST_RPM);
}

// ============================================================
// Loop principal
// ============================================================
void loop() {
  updateMotor();
  calculateSpeed();
  updateCurrentReading();

  ws.cleanupClients();

  if (!isWebConnected) {
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_BUILTIN_PIN, !digitalRead(LED_BUILTIN_PIN));
    }
  }

  if (isWebConnected && (millis() - lastWsSendTime > WS_SEND_INTERVAL)) {
    lastWsSendTime = millis();
    broadcastState();
  }

  debugSerial();
  updateLCD();
  yield();
}