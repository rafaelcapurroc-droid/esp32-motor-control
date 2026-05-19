#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <QuickPID.h>
#include "motor_config.h"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- ESTADO DEL MOTOR ---
volatile int targetSetpointPWM = 0;  // Velocidad deseada solicitada (0 a MAX_VELOCITY)
volatile int motorVelocity     = 0;  // PWM real aplicado al motor
volatile int motorDirection    = -1; // Forzado a Reversa
bool isWebConnected = false;
unsigned long lastWsSendTime = 0;

// Corriente
float cachedCurrent = 0.0f;
unsigned long lastCurrentReadTime = 0;

// Debug serial
unsigned long lastDebugTime = 0;
unsigned long lastBlink = 0;

// --- VARIABLES SENSOR HALL ---
volatile uint32_t hallPeriodBuffer[HALL_BUFFER_SIZE] = {0};
volatile uint8_t  hallBufferIndex = 0;
volatile uint8_t  hallBufferCount = 0;
volatile uint32_t lastHallMicros  = 0;

float currentRPM           = 0.0f;
float currentLinearSpeedMs = 0.0f;

// --- PID ---
float pidSetpoint = 0.0f;          // Setpoint en m/s
float pidOutput   = 0.0f;          // Salida PID (PWM)
QuickPID myPID(&currentLinearSpeedMs, &pidOutput, &pidSetpoint);

// --- ENCODER ---
volatile int encoderPosition = 0;
unsigned long lastEncoderReadTime = 0;
volatile bool encoderButtonPressed = false;
unsigned long lastButtonDebounce = 0;

// --- EMERGENCY STOP ---
volatile bool emergencyStopActive = false;
unsigned long lastEmergencyDebounce = 0;

// --- LCD ---
LiquidCrystal_I2C *lcd = nullptr;
int pendingVelocity = 0;
unsigned long lastLcdUpdate = 0;

uint8_t scanI2CAddress() {
    uint8_t foundAddress = 0;
    Serial.println("[I2C] Starting bus scan...");
    for (uint8_t address = 0x20; address < 0x40; address++) {
        Wire.beginTransmission(address);
        if (Wire.endTransmission() == 0) {
            Serial.printf("[I2C] Device found at 0x%02X\n", address);
            if (address == 0x27 || address == 0x3F || address == 0x3E || address == 0x20 || address == 0x38) {
                foundAddress = address;
                break;
            }
            if (!foundAddress) {
                foundAddress = address;
            }
        }
    }
    return foundAddress;
}

void initLCD() {
    if (lcd) return;
    Wire.begin(LCD_SDA_PIN, LCD_SCL_PIN);
    Wire.setClock(400000);
    uint8_t address = scanI2CAddress();
    if (address == 0) {
        Serial.println("[LCD] No I2C devices found on bus 21/22");
        return;
    }
    Serial.printf("[LCD] Using LCD I2C address 0x%02X\n", address);
    lcd = new LiquidCrystal_I2C(address, 16, 2);
    lcd->begin(16, 2);
    lcd->backlight();
    lcd->clear();
    lcd->display();
    Serial.printf("[LCD] Initialized at 0x%02X\n", address);
}

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
// Cálculo de velocidad
// ============================================================
void calculateSpeed() {
    static unsigned long lastCalcTime = 0;
    const unsigned long CALC_INTERVAL_MS = 50;
    if (millis() - lastCalcTime < CALC_INTERVAL_MS) return;
    lastCalcTime = millis();

    uint32_t periodos[HALL_BUFFER_SIZE];
    uint8_t  count;
    uint32_t ultimoPulso;

    portDISABLE_INTERRUPTS();
    memcpy(periodos, (const void*)hallPeriodBuffer, sizeof(periodos));
    count       = hallBufferCount;
    ultimoPulso = lastHallMicros;
    portENABLE_INTERRUPTS();

    bool timeout = ((micros() - ultimoPulso) > HALL_TIMEOUT_US);
    if (timeout || count == 0) {
        currentRPM           = 0.0f;
        currentLinearSpeedMs = 0.0f;
        return;
    }

    uint32_t suma = 0;
    for (uint8_t i = 0; i < HALL_BUFFER_SIZE; i++) {
        suma += periodos[i];
    }
    uint32_t periodoPromedio = suma / HALL_BUFFER_SIZE;

    if (periodoPromedio > 100 && periodoPromedio < 2000000) {
        currentLinearSpeedMs = HALL_CONST_SPEED / (float)periodoPromedio;
        currentRPM           = HALL_CONST_RPM   / (float)periodoPromedio;
    } else {
        currentRPM           = 0.0f;
        currentLinearSpeedMs = 0.0f;
    }
}

// ============================================================
// Setup motor
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
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), onHallSensorTrigger, RISING);

    pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
    pinMode(ENCODER_DT_PIN,  INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), onEncoderInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN),  onEncoderButton,    FALLING);

    pinMode(EMERGENCY_STOP_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(EMERGENCY_STOP_PIN), onEmergencyStop, FALLING);

    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
#if ESP_IDF_VERSION_MAJOR >= 5
    ledcWrite(MOTOR_PWM_PIN, 0);
#else
    ledcWrite(PWM_CHANNEL, 0);
#endif

    myPID.SetTunings(PID_KP, PID_KI, PID_KD);
    myPID.SetOutputLimits(0, MAX_VELOCITY);
    myPID.SetSampleTimeUs(50);
    myPID.SetMode(QuickPID::Control::automatic);
}

void broadcastState();

// ============================================================
// Loop de control
// ============================================================
void updateMotor() {
    if (emergencyStopActive) {
        targetSetpointPWM   = 0;
        motorVelocity       = 0;
        encoderPosition     = 0;
        pidSetpoint         = 0.0f;
        pidOutput           = 0.0f;
        emergencyStopActive = false;
        myPID.SetMode(QuickPID::Control::manual);
        pidOutput = 0.0f;
        myPID.SetMode(QuickPID::Control::automatic);
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
#if ESP_IDF_VERSION_MAJOR >= 5
        ledcWrite(MOTOR_PWM_PIN, 0);
#else
        ledcWrite(PWM_CHANNEL, 0);
#endif
        broadcastState();
        return;
    }

    if (encoderButtonPressed) {
        encoderButtonPressed = false;
        targetSetpointPWM = pendingVelocity;
        encoderPosition   = pendingVelocity;
        // Al confirmar con el encoder, actualizamos el setpoint del PID inmediatamente
        pidSetpoint = ((float)targetSetpointPWM / (float)MAX_VELOCITY) * MAX_SPEED_MS;
    }

    if (targetSetpointPWM == 0) {
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
        pidSetpoint   = 0.0f;
        pidOutput     = 0.0f;
        motorVelocity = 0;
    } else {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
        
        // Traducimos el setpoint PWM deseado por el usuario a m/s lineales
        pidSetpoint = ((float)targetSetpointPWM / (float)MAX_VELOCITY) * MAX_SPEED_MS;
        
        myPID.Compute();
        motorVelocity = constrain((int)pidOutput, 0, MAX_VELOCITY);
    }

#if ESP_IDF_VERSION_MAJOR >= 5
    ledcWrite(MOTOR_PWM_PIN, motorVelocity);
#else
    ledcWrite(PWM_CHANNEL, motorVelocity);
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
    doc["target_pwm"]      = targetSetpointPWM;
    doc["pendingVelocity"] = pendingVelocity;
    doc["direction"]       = motorDirection; 
    doc["ledState"]        = (digitalRead(LED_BUILTIN_PIN) == HIGH);
    doc["current"]         = cachedCurrent;
    doc["rpm"]             = currentRPM;
    doc["speed_m_s"]       = currentLinearSpeedMs; // Velocidad REAL (Sensor Hall)
    
    // Calculamos la velocidad objetivo en m/s basada en el PWM target actual
    float targetSpeedMs = ((float)targetSetpointPWM / (float)MAX_VELOCITY) * MAX_SPEED_MS;
    doc["target_speed"]  = targetSpeedMs; // <--- ESTO ES LO QUE SINCRONIZA LA WEB
    
    doc["pid_output"]    = (int)pidOutput;
    
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
    Serial.printf("[PID] Target: %.2f m/s | Sensed: %.2f m/s | Applied PWM: %d | Dir: %d\n", 
                  pidSetpoint, currentLinearSpeedMs, motorVelocity, motorDirection);
    Serial.printf("[HALL] RPM: %.1f | Current: %.2f A | EncoderPos: %d | WS: %u\n",
                  currentRPM, cachedCurrent, encoderPosition, ws.count());
}

// ============================================================
// LCD Functions
// ============================================================
void lcdShowWelcome() {
    if (!lcd) return;
    lcd->clear();
    lcd->setCursor(0, 0);
    lcd->print("  Motor Control ");
    lcd->setCursor(0, 1);
    lcd->print(" Rev-Only Mode  ");
    delay(2500);
    lcd->clear();
}

void updateLCD() {
    if (!lcd) return;
    if (millis() - lastLcdUpdate < LCD_UPDATE_INTERVAL) return;
    lastLcdUpdate = millis();

    lcd->setCursor(1, 0);
    lcd->print("Sen:");
    char bufSen[12];
    snprintf(bufSen, sizeof(bufSen), "%-6.2f", currentLinearSpeedMs);
    lcd->print(bufSen);
    lcd->print("m/s ");

    lcd->setCursor(1, 1);
    if (pendingVelocity != targetSetpointPWM) {
        float previewSpeed = ((float)pendingVelocity / (float)MAX_VELOCITY) * MAX_SPEED_MS;
        lcd->print("Pre:");
        char bufPre[12];
        snprintf(bufPre, sizeof(bufPre), "%-5.2f*", previewSpeed);
        lcd->print(bufPre);
        lcd->print("m/s");
    } else {
        lcd->print("Set:");
        char bufDes[12];
        snprintf(bufDes, sizeof(bufDes), "%-6.2f", pidSetpoint);
        lcd->print(bufDes);
        lcd->print("m/s ");
    }
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

            // Soporte: recibir velocidad directamente en m/s o mm/s
            if (doc.containsKey("speed_m_s")) {
                float spd = doc["speed_m_s"].as<float>();
                if (spd < 0.0f) spd = 0.0f;
                if (spd > MAX_SPEED_MS) spd = MAX_SPEED_MS;
                
                pidSetpoint = spd;
                int pwm = (int)(((spd / MAX_SPEED_MS) * (float)MAX_VELOCITY) + 0.5f);
                pwm = constrain(pwm, 0, MAX_VELOCITY);
                
                targetSetpointPWM = pwm;
                encoderPosition = pwm;
                pendingVelocity = pwm;
                stateChanged = true;
                
            } else if (doc.containsKey("speed_mm_s")) {
                float spd = doc["speed_mm_s"].as<float>() / 1000.0f;
                if (spd < 0.0f) spd = 0.0f;
                if (spd > MAX_SPEED_MS) spd = MAX_SPEED_MS;
                
                pidSetpoint = spd;
                int pwm = (int)(((spd / MAX_SPEED_MS) * (float)MAX_VELOCITY) + 0.5f);
                pwm = constrain(pwm, 0, MAX_VELOCITY);
                
                targetSetpointPWM = pwm;
                encoderPosition = pwm;
                pendingVelocity = pwm;
                stateChanged = true;
            }
            
            if (doc["velocity"].is<int>()) {
                int vel = constrain(doc["velocity"].as<int>(), 0, MAX_VELOCITY);
                if (vel != targetSetpointPWM) {
                    targetSetpointPWM = vel;
                    encoderPosition   = vel;
                    pendingVelocity   = vel;
                    
                    // Actualizar también el setpoint del PID en m/s para consistencia interna
                    pidSetpoint = ((float)vel / (float)MAX_VELOCITY) * MAX_SPEED_MS;
                    
                    stateChanged = true;
                }
            }

            if (doc["direction"].is<int>()) {
                motorDirection = -1;
                stateChanged   = true;
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
<title>ESP32 Motor - Diagnostico (REV Only)</title>
<style>
body{font-family:monospace;background:#0d0f14;color:#c8d0e0;padding:30px;max-width:600px;margin:0 auto}
h2{color:#00e5ff;margin-bottom:6px}
.warn{color:#ff6b35;background:#1e1500;border:1px solid #ff6b35;padding:12px;border-radius:6px;margin:16px 0}
.ok{color:#39ff6e}
code{background:#1e2330;padding:2px 6px;border-radius:3px;font-size:0.9em}
.metric{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #1e2330}
.metric:last-child{border-bottom:none}
#ws-status{color:#ff3355}
</style>
</head><body>
<h2>ESP32 Motor Control [Modo Reversa Unica]</h2>
<p>Modo diagnostico - <span id="ws-status">sin conexion WS</span></p>
<div class="warn">
<strong>Archivos no encontrados en LittleFS</strong><br>
La interfaz principal (index.html) no esta en el sistema de archivos del ESP32.<br><br>
<strong>Solucion:</strong><br>
Ejecuta: <code>pio run --target uploadfs</code>
</div>
<h3 style="color:#00e5ff;margin-top:20px">Estado del motor</h3>
<div id="metrics">
<div class="metric"><span>RPM</span><span id="m-rpm" class="ok">-</span></div>
<div class="metric"><span>Velocidad</span><span id="m-spd" class="ok">-</span></div>
<div class="metric"><span>Corriente</span><span id="m-cur" style="color:#ff6b35">-</span></div>
<div class="metric"><span>PWM Real (PID)</span><span id="m-pwm">-</span></div>
<div class="metric"><span>Direccion</span><span id="m-dir" style="color:#ff6b35">REV FIXED</span></div>
</div>
<h3 style="color:#00e5ff;margin-top:20px">Control de Velocidad</h3>
<label>Velocidad Target (0-2047):<br>
<input type="range" min="0" max="2047" value="0" id="sl" style="width:100%;margin:8px 0"
oninput="send({velocity:parseInt(this.value)})">
</label>
<div style="display:flex;gap:8px;margin-top:8px">
<button onclick="send({direction:-1})"  style="flex:1;padding:12px;background:#1e2330;color:#ff6b35;border:1px solid #ff6b35;border-radius:4px;cursor:pointer;font-weight:bold">Habilitar Marcha REVERSA</button>
<button onclick="send({velocity:0})"    style="flex:1;padding:12px;background:#1e2330;color:#ff3355;border:1px solid #ff3355;border-radius:4px;cursor:pointer;font-weight:bold">STOP</button>
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
if(d.velocity!==undefined)document.getElementById('m-pwm').textContent=d.velocity+' / 2047';
if(d.target_pwm!==undefined)document.getElementById('sl').value=d.target_pwm;
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
    analogReadResolution(12);
    
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] ERROR: No se pudo inicializar LittleFS");
    } else {
        Serial.println("[FS] LittleFS OK");
    }
    
    setupMotor();
    initLCD();
    lcdShowWelcome();
    
    pendingVelocity = targetSetpointPWM;
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_AP_PASSWORD);
    Serial.printf("[WiFi] AP: %s | IP: %s\n", WIFI_SSID, WiFi.softAPIP().toString().c_str());
    
    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    setupHTTPEndpoints();
    server.begin();
    Serial.println("[HTTP] Servidor iniciado");
}

// ============================================================
// Loop principal
// ============================================================
void loop() {
    calculateSpeed();
    updateMotor();
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