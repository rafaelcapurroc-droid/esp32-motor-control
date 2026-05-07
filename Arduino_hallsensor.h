/*
 * Medidor de Velocidad de Cinta Transportadora - Arduino Uno
 * Autor: Rafael
 * 
 * CONFIGURACIÓN MECÁNICA:
 * - Rodillo motriz (Cinta): Ø 40 mm
 * - Polea sensora (Eje motriz): Ø 60 mm con 8 imanes
 * - Sensor Hall: Pin 2
 */

#include <Arduino.h>

// --- CONFIGURACIÓN DE PINES ---
#define HALL_PIN 2  // Interrupción 0 en Arduino Uno

// --- CONSTANTES DEL SISTEMA ---
#define IMANES_COUNT 8
#define RODILLO_DIAMETRO_MM 40.0

// --- CÁLCULO DE CONSTANTES ---
// Perímetro del rodillo en metros: (PI * 40mm) / 1000 = 0.1256637 m
const float PERIMETRO_RODILLO_M = (PI * RODILLO_DIAMETRO_MM) / 1000.0;

// Factor para convertir Frecuencia (Hz) a m/s:
// V(m/s) = (Perimetro / Imanes) * Frecuencia(Hz)
// Factor = 0.1256637 / 8 = 0.01570796
const float FACTOR_VELOCIDAD = PERIMETRO_RODILLO_M / IMANES_COUNT;

// --- VARIABLES VOLÁTILES (INTERRUPCIÓN) ---
volatile unsigned long ultimoTiempoMicros = 0;
volatile unsigned long periodoMicros = 0; // Tiempo entre dos pulsos consecutivos
volatile bool nuevoDato = false;

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL_PIN, INPUT_PULLUP);
  
  // Interrupción en flanco de bajada
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), ISR_Hall, FALLING);
  
  Serial.println("Sistema listo. Midiendo velocidad de cinta...");
  Serial.println("Formato: [m/s] | [RPM Eje]");
}

void loop() {
  static unsigned long lastPrint = 0;
  
  // Actualizar pantalla cada 500ms para tener lectura estable
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    
    float velocidadMs = 0.0;
    float rpmEje = 0.0;
    
    // Zona crítica: leer datos protegidos
    noInterrupts();
    unsigned long periodoActual = periodoMicros;
    interrupts();
    
    // Si hay periodo válido (mayor a 0 y menor a 2 segundos)
    if (periodoActual > 100 && periodoActual < 2000000) {
      
      // 1. Calcular Velocidad Lineal (m/s)
      // Fórmula: (Perimetro / Imanes) / Periodo(segundos)
      // Usamos micros: Factor * 1,000,000 / periodoMicros
      velocidadMs = (FACTOR_VELOCIDAD * 1000000.0) / periodoActual;
      
      // 2. Calcular RPM del Eje Motriz
      // RPM = (60 segundos * 1,000,000 micros) / (Periodo * Imanes)
      rpmEje = 60000000.0 / (periodoActual * IMANES_COUNT);
    }
    
    // Imprimir resultados
    Serial.print("Velocidad: ");
    Serial.print(velocidadMs, 3); // 3 decimales (ej: 0.523 m/s)
    Serial.print(" m/s  |  RPM Eje: ");
    Serial.println(rpmEje, 1);    // 1 decimal (ej: 250.5 RPM)
  }
}

// --- INTERRUPCIÓN ---
void ISR_Hall() {
  unsigned long ahora = micros();
  
  // Debounce por tiempo: ignorar rebotes < 1ms (1000us)
  // Esto filtra ruido eléctrico común en motores
  if (ahora - ultimoTiempoMicros > 1000) {
    periodoMicros = ahora - ultimoTiempoMicros;
    ultimoTiempoMicros = ahora;
  }
}
