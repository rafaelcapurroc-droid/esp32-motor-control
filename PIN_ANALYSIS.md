# ESP32 Pin Assignment Analysis

## Verificación de Pines - Sin Conflictos ✅

### Pines Actuales (Motor Control + Current Sense)

| Pin | Función | Tipo | Uso | Estado |
|-----|---------|------|-----|--------|
| GPIO 25 | Motor PWM | PWM/IO | Speed Control | ✅ OK |
| GPIO 26 | Motor IN1 | IO | Direction Fwd | ✅ OK |
| GPIO 27 | Motor IN2 | IO | Direction Rev | ✅ OK |
| GPIO 35 | Current Sense | ADC1_7 | CS (input only) | ✅ OK |

### WiFi - Pines Reservados (No usar)

| Pin | Razón | Notas |
|-----|-------|-------|
| GPIO 1, 3 | UART Debug | Serial console |
| GPIO 6-11 | SPI Flash | Memoria interna |
| GPIO 0 | Boot selection | Evitar |
| GPIO 2 | D2 LED / Boot | Evitar para WiFi |
| GPIO 4, 5 | Posibles issues WiFi | No óptimo |
| GPIO 12 | JTAG / Boot | Evitar |
| GPIO 13, 14, 15 | JTAG | Evitar para WiFi |
| GPIO 23, 18, 19 | SPI (opcional) | Si usas SPI |

### ✅ Pines Seguros para Nuevas Funciones

Disponibles sin conflictos WiFi:

| Pin | Tipo | Voltaje | Recomendación |
|-----|------|---------|---------------|
| GPIO 32 | ADC1_4 | 0-3.3V | ✅ **Mejor** para Potenciómetro |
| GPIO 33 | ADC1_5 | 0-3.3V | ✅ **Alternativa** |
| GPIO 34 | ADC1_6 | 0-3.3V | ⚠️ Input only |
| GPIO 36 | ADC1_0 | 0-3.3V | ⚠️ Problemas WiFi |
| GPIO 39 | ADC1_3 | 0-3.3V | ⚠️ Problemas WiFi |

---

## Solución Recomendada

### Para Potenciometro: **GPIO 32**

```cpp
#define POTENTIOMETER_PIN 32  // ADC1_4 - 100% seguro
```

**Razones**:
- ✅ Totalmente independent (no usado por WiFi)
- ✅ ADC1_4 confiable
- ✅ Entrada análoga 0-3.3V perfecta para potenciómetro
- ✅ No interfiere con PWM ni dirección

### Conexión del Potenciometro

```
Potenciometro (10K ohm típico):
  Pin 1 (G)  → GND
  Pin 2 (W)  → GPIO 32 (ADC1_4)
  Pin 3 (+)  → 3.3V
```

---

## Resumen - Configuración Final

### Pines Motor Control
```cpp
GPIO 25 → Motor PWM (20kHz)
GPIO 26 → Motor IN1 (dirección forward)
GPIO 27 → Motor IN2 (dirección reverse)
GPIO 35 → Current Sense (ADC - input only)
```

### Nuevos Pines
```cpp
GPIO 32 → Potenciómetro (ADC - entrada análoga)
```

### WiFi
```
STA/AP Mode: GPIO no específicos
LED Wi-Fi: Interno (no visible)
```

---

## ✅ CONCLUSIÓN: Sin Conflictos

- ✅ GPIO 25-27 OK para motor PWM + dirección
- ✅ GPIO 35 OK para corriente (input only ADC)
- ✅ GPIO 32 OK para potenciometro (ADC independiente)
- ✅ WiFi opera normalmente (no usa GPIO 25, 26, 27, 32, 35)
- ✅ **Configuración segura y funcional**

---

## Versiones de ESP32 Soportadas

| Modelo | WiFi | SPI Flash | RAM | Recomendación |
|--------|------|-----------|-----|---------------|
| ESP32-WROOM-32 | ✅ 2.4GHz | ✅ 4MB | ✅ 520KB | ✅ **Soportado** |
| ESP32-D0WD | ✅ 2.4GHz | ✅ 4MB | ✅ 520KB | ✅ **Soportado** |
| ESP32-S2 | ✅ WiFi | ✅ | ✅ | ⚠️ Menos RAM |
| ESP32-S3 | ✅ WiFi+BLE | ✅ | ✅ Mejor | ✅ **Compatible** |

---

## Diagrama de Pines

```
ESP32 DevKit V1
┌─────────────────────────────────┐
│ GND    GPIO0   GPIO2   GPIO4    │
│ GPIO34 GPIO35  GPIO32  GPIO33   │ ← GPIO 35 (Current Sense)
│ GPIO27 GPIO26  GPIO25  GPIO24   │ ← GPIO 25 (PWM), GPIO 26/27 (Direction)
│ GPIO23 GPIO22  GPIO21  GPIO20   │
│ ...                             │
│ 3.3V   GND     EN    GPIO39     │
└─────────────────────────────────┘
                  ↑
            GPIO 32 (Potenciómetro)
```

---

## Calibración ADC

### GPIO 35 (Current Sense)
- Rango: 0-4095 (12-bit)
- Voltaje: 0-3.3V
- Escala: 1V = 1A (VNH5019)
- Fórmula: `I = (ADC * 3.3 / 4095 - 0.5) / 1.0`

### GPIO 32 (Potenciometro)
- Rango: 0-4095 (12-bit)
- Voltaje: 0-3.3V
- Escala: 0 → 0% velocidad, 4095 → 100% velocidad
- Fórmula: `velocity = (ADC / 4095) * 255`

---

## Recomendación Final

✅ **La configuración actual es segura**
✅ **GPIO 32 es ideal para potenciometro**
✅ **No hay conflictos ni interferencias**
✅ **Listo para implementar**
