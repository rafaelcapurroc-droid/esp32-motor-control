#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>          // FIX: SPIFFS está deprecado, usar LittleFS
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

#define LED_BUILTIN_PIN 2      // FIX: nombre explícito para evitar conflicto con macro
unsigned long lastBlink = 0;

// --- VARIABLES SENSOR HALL ---
// FIX: todas las variables compartidas con ISR son volatile
volatile unsigned long lastHallMicros   = 0;
volatile unsigned long prevHallMicros   = 0;   // FIX: movida a volatile (antes era estática local)
volatile bool          newPulseAvailable = false;
volatile unsigned long lastPulseMs      = 0;   // FIX: movida a volatile (antes era estática local)

float currentRPM          = 0.0f;
float currentLinearSpeedMs = 0.0f;

portMUX_TYPE hallMutex = portMUX_INITIALIZER_UNLOCKED;

// --- VARIABLES ENCODER KY-040 ---
volatile int encoderPosition = 0;
unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_DEBOUNCE_MS = 5;

// --- BOTÓN ENCODER ---
volatile bool encoderButtonPressed = false;
unsigned long lastButtonDebounce = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

// ============================================================
// ISRs
// ============================================================

void IRAM_ATTR onHallSensorTrigger() {
    unsigned long now = micros();
    // FIX: usar constante definida en motor_config.h
    if ((now - lastHallMicros) > HALL_MIN_PULSE_US) {
        prevHallMicros   = lastHallMicros;   // guardar pulso anterior (volátil)
        lastHallMicros   = now;
        lastPulseMs      = millis();
        newPulseAvailable = true;
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
            encoderPosition += ENCODER_VELOCITY_STEP;
        } else {
            encoderPosition -= ENCODER_VELOCITY_STEP;
        }
    }

    if (encoderPosition > MAX_VELOCITY) encoderPosition = MAX_VELOCITY;
    if (encoderPosition < 0)            encoderPosition = 0;
}

void IRAM_ATTR onEncoderButton() {
    unsigned long now = millis();
    if (now - lastButtonDebounce < BUTTON_DEBOUNCE_MS) return;
    lastButtonDebounce   = now;
    encoderButtonPressed = true;
}

// ============================================================
// Cálculo de velocidad
// ============================================================

void calculateSpeed() {
    static unsigned long lastCalcTime = 0;

    if (millis() - lastCalcTime < 50) return;
    lastCalcTime = millis();

    // FIX: leer todas las variables volátiles dentro de la sección crítica
    portENTER_CRITICAL(&hallMutex);
    bool          hasNewPulse    = newPulseAvailable;
    unsigned long currentMicros  = lastHallMicros;
    unsigned long previousMicros = prevHallMicros;
    unsigned long pulseMs        = lastPulseMs;
    newPulseAvailable = false;
    portEXIT_CRITICAL(&hallMutex);

    if (hasNewPulse && previousMicros != 0) {
        unsigned long deltaTime = currentMicros - previousMicros;

        if (deltaTime > 1000UL && deltaTime < 2000000UL) {
            float newRPM = 60000000.0f / ((float)deltaTime * MAGNETS_COUNT);

            // Filtro de salto brusco (ratio > 3x o < 0.33x)
            if (currentRPM > 10.0f) {
                float ratio = newRPM / currentRPM;
                if (ratio > 3.0f || ratio < 0.33f) {
                    currentLinearSpeedMs = currentRPM * M_S_PER_RPM;
                    return;
                }
            }
            // Filtro EMA
            currentRPM = (newRPM * 0.6f) + (currentRPM * 0.4f);
        }
    } else if (currentRPM > 0.0f && (millis() - pulseMs > HALL_TIMEOUT_MS)) {
        // Sin pulsos recientes → motor detenido
        currentRPM = 0.0f;
    }

    currentLinearSpeedMs = currentRPM * M_S_PER_RPM;
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

    // Sensor Hall
    pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN),
                    onHallSensorTrigger, FALLING);

    // Encoder KY-040
    pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
    pinMode(ENCODER_DT_PIN,  INPUT_PULLUP);
    pinMode(ENCODER_SW_PIN,  INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN),
                    onEncoderInterrupt, FALLING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN),
                    onEncoderButton, FALLING);

    // Estado inicial — motor frenado
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
    // Sincronizar posición del encoder con velocidad
    if (encoderPosition != motorVelocity) {
        motorVelocity = encoderPosition;
    }

    // Botón del encoder → parada de emergencia
    if (encoderButtonPressed) {
        encoderButtonPressed = false;
        motorVelocity   = 0;
        encoderPosition = 0;
    }

    // Dirección
    if (motorDirection > 0) {
        digitalWrite(MOTOR_IN1_PIN, HIGH);
        digitalWrite(MOTOR_IN2_PIN, LOW);
    } else if (motorDirection < 0) {
        digitalWrite(MOTOR_IN1_PIN, LOW);
        digitalWrite(MOTOR_IN2_PIN, HIGH);
    } else {
        // Freno activo
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
    doc["velocity"]   = motorVelocity;
    doc["direction"]  = motorDirection;
    doc["ledState"]   = (digitalRead(LED_BUILTIN_PIN) == HIGH);
    doc["current"]    = cachedCurrent;
    doc["rpm"]        = currentRPM;
    // FIX: campo unificado — enviamos m/s (la web principal lo espera como speed_m_s)
    doc["speed_m_s"]  = currentLinearSpeedMs;
    // Mantenemos speed_mm_s para compatibilidad con clientes legacy
    doc["speed_mm_s"] = currentLinearSpeedMs * 1000.0f;

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

    // FIX: añadido \n al final para que cada línea aparezca por separado
    Serial.printf("[HALL] RPM: %.1f | Belt: %.3f m/s | Current: %.2f A | PWM: %d | Encoder: %d\n",
                  currentRPM, currentLinearSpeedMs, cachedCurrent,
                  motorVelocity, encoderPosition);
}

// ============================================================
// WebSocket — handler de eventos
// ============================================================

void onWebSocketEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                      AwsEventType type, void* arg, uint8_t* data, size_t len) {

    if (type == WS_EVT_CONNECT) {
        // FIX: límite correcto — rechazar si ya hay 2 o más clientes conectados
        if (ws.count() >= 2) {
            client->close();
            return;
        }
        Serial.printf("[WS] Cliente conectado (id=%u)\n", client->id());
        isWebConnected = true;
        digitalWrite(LED_BUILTIN_PIN, HIGH);
        broadcastState();

    } else if (type == WS_EVT_DISCONNECT) {
        Serial.printf("[WS] Cliente desconectado (id=%u)\n", client->id());
        if (server->count() == 0) {
            isWebConnected = false;
            digitalWrite(LED_BUILTIN_PIN, LOW);
        }

    } else if (type == WS_EVT_DATA) {
        AwsFrameInfo* info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 &&
            info->len == len && info->opcode == WS_TEXT) {

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
// HTTP — rutas estáticas
// ============================================================

void setupHTTPEndpoints() {
    // FIX: usar LittleFS en lugar de SPIFFS
    server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
    server.serveStatic("/js/", LittleFS, "/js/");

    server.onNotFound([](AsyncWebServerRequest* request) {
        request->send(404, "text/plain", "Not Found");
    });
}

// ============================================================
// Setup
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    // FIX: inicializar LittleFS en lugar de SPIFFS
    if (!LittleFS.begin(true)) {
        Serial.println("[FS] Error al inicializar LittleFS");
    } else {
        Serial.println("[FS] LittleFS inicializado correctamente");
    }

    setupMotor();

    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid, apPassword);
    Serial.printf("[WiFi] AP: %s | IP: %s\n",
                  ssid, WiFi.softAPIP().toString().c_str());

    ws.onEvent(onWebSocketEvent);
    server.addHandler(&ws);
    setupHTTPEndpoints();
    server.begin();

    Serial.println("[HTTP] Servidor iniciado");
    Serial.printf("[INFO] 1 RPM = %.6f m/s de cinta\n", M_S_PER_RPM);
}

// ============================================================
// Loop principal
// ============================================================

void loop() {
    updateMotor();
    calculateSpeed();
    updateCurrentReading();
    ws.cleanupClients();

    // Parpadeo LED cuando no hay cliente conectado
    if (!isWebConnected) {
        if (millis() - lastBlink > 500) {
            lastBlink = millis();
            digitalWrite(LED_BUILTIN_PIN,
                         !digitalRead(LED_BUILTIN_PIN));
        }
    }

    // Envío periódico de estado por WebSocket
    if (isWebConnected && (millis() - lastWsSendTime > WS_SEND_INTERVAL)) {
        lastWsSendTime = millis();
        broadcastState();
    }

    debugSerial();
    yield();
}
