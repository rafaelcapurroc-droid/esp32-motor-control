#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "motor_config.h"

const char* ssid = "ESP32_Motor_Control";
const char* apPassword = "12345678";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- ESTADO DEL MOTOR ---
volatile int motorVelocity = 0;
volatile int motorDirection = 1;
bool isWebConnected = false;

unsigned long lastWsSendTime = 0;
const unsigned long WS_SEND_INTERVAL = 200;

// Corriente
float cachedCurrent = 0.0;
unsigned long lastCurrentReadTime = 0;
const unsigned long CURRENT_READ_INTERVAL = 50;

// Debug serial
unsigned long lastDebugTime = 0;
const unsigned long DEBUG_INTERVAL = 1000;

#define LED_BUILTIN 2
unsigned long lastBlink = 0;

// --- VARIABLES SENSOR HALL ---
volatile unsigned long lastHallMicros = 0;
volatile bool newPulseAvailable = false;

float currentRPM = 0.0;
float currentLinearSpeedMs = 0.0;

portMUX_TYPE hallMutex = portMUX_INITIALIZER_UNLOCKED;

// --- VARIABLES ENCODER KY-040 ---
volatile int encoderPosition = 0;
unsigned long lastEncoderReadTime = 0;
const unsigned long ENCODER_DEBOUNCE_MS = 5;

// --- BOTÓN ENCODER ---
volatile bool encoderButtonPressed = false;
unsigned long lastButtonDebounce = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;

void IRAM_ATTR onHallSensorTrigger() {
  unsigned long now = micros();
  if ((now - lastHallMicros) > 100) {
    lastHallMicros = now;
    newPulseAvailable = true;
  }
}

void IRAM_ATTR onEncoderInterrupt() {
  unsigned long now = millis();
  if (now - lastEncoderReadTime < ENCODER_DEBOUNCE_MS) return;
  lastEncoderReadTime = now;

  int clk = digitalRead(ENCODER_CLK_PIN);
  int dt = digitalRead(ENCODER_DT_PIN);

  if (clk == LOW) {
    if (dt == HIGH) {
      encoderPosition += ENCODER_VELOCITY_STEP;
    } else {
      encoderPosition -= ENCODER_VELOCITY_STEP;
    }
  }

  if (encoderPosition > MAX_VELOCITY) encoderPosition = MAX_VELOCITY;
  if (encoderPosition < 0) encoderPosition = 0;
}

void IRAM_ATTR onEncoderButton() {
  unsigned long now = millis();
  if (now - lastButtonDebounce < BUTTON_DEBOUNCE_MS) return;
  lastButtonDebounce = now;
  encoderButtonPressed = true;
}

void calculateSpeed() {
  static unsigned long lastCalcTime = 0;
  static unsigned long previousMicros = 0;
  static unsigned long lastPulseMs = 0;

  if (millis() - lastCalcTime >= 50) {
    portENTER_CRITICAL(&hallMutex);
    bool hasNewPulse = newPulseAvailable;
    unsigned long currentMicros = lastHallMicros;
    newPulseAvailable = false;
    portEXIT_CRITICAL(&hallMutex);

    if (hasNewPulse) {
      if (previousMicros != 0) {
        unsigned long deltaTime = currentMicros - previousMicros;
        if (deltaTime > 1000 && deltaTime < 2000000) {
          float newRPM = 60000000.0 / ((float)deltaTime * MAGNETS_COUNT);

          if (currentRPM > 10.0) {
            float ratio = newRPM / currentRPM;
            if (ratio > 3.0 || ratio < 0.33) {
              previousMicros = currentMicros;
              lastPulseMs = millis();
              lastCalcTime = millis();
              return;
            }
          }
          currentRPM = (newRPM * 0.6) + (currentRPM * 0.4);
        }
      }
      previousMicros = currentMicros;
      lastPulseMs = millis();
    } else {
      if (currentRPM > 0 && (millis() - lastPulseMs > 400)) {
        currentRPM = 0;
        previousMicros = 0;
      }
    }

    currentLinearSpeedMs = currentRPM * M_S_PER_RPM;
    lastCalcTime = millis();
  }
}

void setupMotor() {
  #if ESP_IDF_VERSION_MAJOR >= 5
    ledcAttach(MOTOR_PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);
  #else
    ledcSetup(PWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
    ledcAttachPin(MOTOR_PWM_PIN, PWM_CHANNEL);
  #endif

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_CS_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(HALL_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_SENSOR_PIN), onHallSensorTrigger, FALLING);

  pinMode(ENCODER_CLK_PIN, INPUT_PULLUP);
  pinMode(ENCODER_DT_PIN, INPUT_PULLUP);
  pinMode(ENCODER_SW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK_PIN), onEncoderInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_SW_PIN), onEncoderButton, FALLING);

  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
  #if ESP_IDF_VERSION_MAJOR >= 5
    ledcWrite(MOTOR_PWM_PIN, 0);
  #else
    ledcWrite(PWM_CHANNEL, 0);
  #endif
}

void updateMotor() {
  if (encoderPosition != motorVelocity) {
    motorVelocity = encoderPosition;
  }

  if (encoderButtonPressed) {
    encoderButtonPressed = false;
    motorVelocity = 0;
    encoderPosition = 0;
  }

  if (motorDirection > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else if (motorDirection < 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  } else {
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

void updateCurrentReading() {
  if (millis() - lastCurrentReadTime < CURRENT_READ_INTERVAL) return;
  lastCurrentReadTime = millis();

  uint32_t sum = 0;
  const int samples = 20;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(MOTOR_CS_PIN);
  }

  float avgRaw = sum / (float)samples;
  float voltage = avgRaw * (3.3f / 4095.0f);
  float correctedVoltage = voltage - CS_OFFSET_VOLTAGE;
  if (correctedVoltage < 0) correctedVoltage = 0;
  cachedCurrent = correctedVoltage / CS_VOLTAGE_PER_AMP;
}

void broadcastState() {
  if (ws.count() == 0) return;

  JsonDocument doc;
  doc["velocity"]      = motorVelocity;
  doc["direction"]     = motorDirection;
  doc["ledState"]      = (digitalRead(LED_BUILTIN) == HIGH);
  doc["current"]       = cachedCurrent;
  doc["rpm"]           = currentRPM;
  doc["speed_mm_s"]    = currentLinearSpeedMs * 1000.0;  // Convertir m/s a mm/s

  String json;
  serializeJson(doc, json);
  ws.textAll(json);
}

void debugSerial() {
  if (millis() - lastDebugTime < DEBUG_INTERVAL) return;
  lastDebugTime = millis();

  Serial.printf("[HALL] RPM: %.1f | Belt: %.3f m/s | Current: %.2f A | PWM: %d | Encoder: %d", currentRPM, currentLinearSpeedMs, cachedCurrent, motorVelocity, encoderPosition);
}

void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    if (ws.count() > 2) {
      client->close();
      return;
    }
    Serial.println("[WS] Cliente Conectado");
    isWebConnected = true;
    digitalWrite(LED_BUILTIN, HIGH);
    broadcastState();

  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("[WS] Cliente Desconectado");
    if (server->count() == 0) {
      isWebConnected = false;
      digitalWrite(LED_BUILTIN, LOW);
    }

  } else if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {

      JsonDocument doc;
      if (deserializeJson(doc, data, len)) return;

      bool stateChanged = false;

      if (doc["led"].is<bool>()) {
        digitalWrite(LED_BUILTIN, doc["led"].as<bool>() ? HIGH : LOW);
      }

      if (doc["velocity"].is<int>()) {
        int vel = doc["velocity"].as<int>();
        vel = constrain(vel, 0, MAX_VELOCITY);
        if (vel != motorVelocity) {
          motorVelocity = vel;
          encoderPosition = vel;
          stateChanged = true;
        }
      }

      if (doc["direction"].is<int>()) {
        int dir = doc["direction"].as<int>();
        int newDir = (dir >= 0) ? 1 : -1;
        if (newDir != motorDirection) {
          motorDirection = newDir;
          stateChanged = true;
        }
      }

      if (stateChanged) {
         broadcastState();
      }
    }
  }
}

void setupHTTPEndpoints() {
  // Servir archivos estáticos desde SPIFFS
  server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  server.serveStatic("/js/", SPIFFS, "/js/");
  
  // Ruta para archivos no encontrados
  server.onNotFound([](AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not Found");
  });
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Inicializar SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("Error al inicializar SPIFFS");
  } else {
    Serial.println("SPIFFS inicializado correctamente");
  }

  setupMotor();

  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, apPassword);
  Serial.print("IP AP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  setupHTTPEndpoints();
  server.begin();

  Serial.println("Sistema listo. KY-040 activo.");
  Serial.printf("Relacion: 1 RPM = %.6f m/s de cinta", M_S_PER_RPM);
}

void loop() {
  updateMotor();
  calculateSpeed();
  updateCurrentReading();
  ws.cleanupClients();

  if (!isWebConnected) {
    if (millis() - lastBlink > 500) {
      lastBlink = millis();
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
  }

  if (isWebConnected && (millis() - lastWsSendTime > WS_SEND_INTERVAL)) {
    lastWsSendTime = millis();
    broadcastState();
  }

  debugSerial();
  yield();
}