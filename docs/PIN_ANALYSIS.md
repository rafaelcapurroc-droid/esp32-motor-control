# ESP32 Pin Assignment Analysis

## Verificación de Pines - Sin Conflictos ✅

### Pines Actuales (Motor Control + Current Sense)

| Pin | Función | Tipo | Uso | Estado |
| --- | --- | --- | --- | --- |
| GPIO 25 | MOTOR_PWM_PIN | PWM/IO | Motor Speed Control | ✅ OK |
| GPIO 26 | MOTOR_IN1_PIN | IO | Motor Direction Forward | ✅ OK |
| GPIO 27 | MOTOR_IN2_PIN | IO | Motor Direction Reverse | ✅ OK |
| GPIO 35 | MOTOR_CS_PIN | ADC1_7 | Current Sense (input only) | ✅ OK |
| GPIO 33 | HALL_SENSOR_PIN | IO | Hall Sensor (speed feedback) | ✅ OK |
| GPIO 2 | LED_BUILTIN_PIN | IO | Built-in LED indicator | ✅ OK |

### Pines Adicionales - Periféricos

| Pin | Función | Tipo | Uso | Estado |
| --- | --- | --- |
| GPIO 1, 3 | UART Debug | Serial console |
| GPIO 6-11 | SPI Flash | Memoria interna |
| GPIO 0 | Boot selection | Evitar |
| GPIO 4, 5 | Posibles issues WiFi | No óptimo |
| GPIO 12 | JTAG / Boot | Evitar |
| GPIO 14, 15 | JTAG | Evitar para WiFi

| Pin | Razón | Notas |
| ----- | ------- | ------- |
| GPIO 1, 3 | UART Debug | Serial console |
| GPIO 6-11 | SPI Flash | Memoria interna |
| GPIO 0 | Boot selection | Evitar |
| GPIO 2 | D2 LED / Boot | Evitar para WiFi |
| GPIO 4, 5 | Posibles issues WiFi | No óptimo |
| GPIO 12 | JTAG / Boot | Evitar |
| GPIO 13, 14, 15 | JTAG | Evitar para WiFi |
| GPIO 23, 18, 19 | SPI (opcional) | Si usas SPI |

### ✅ Pines Seguros para Nuevas Funciones

Disponibles Disponibles para Expansión

| Pin | Tipo | Voltaje | Notas |
| --- | --- | --- | --- |
| GPIO 32 | ADC1_4 | 0-3.3V | ✅ Libre para nuevas funciones |
| GPIO 34 | ADC1_6 | 0-3.3V | ✅ Entrada análoga libre |
| GPIO 36 | ADC1_0 | 0-3.3V | ⚠️ Compartido con WiFi |
| GPIO 39 | ADC1_3 | 0-3.3V | ⚠️ Compartido con WiFi |

---

## Resumen - Configuración Final

### Pines Motor Control

```cpp
GPIO 25 → Motor PWM (20kHz)
GPIO 26 → Motor IN1 (dirección forward)
GPIO 27 → Motor IN2 (dirección reverse)
GPIO 35 → Current Sense (ADC - input only)
```

 (VNH5019)

```cpp
GPIO 25 → Motor PWM (20kHz, 10-bit: 0-1023)
GPIO 26 → Motor IN1 (dirección forward)
GPIO 27 → Motor IN2 (dirección reverse)
GPIO 35 → Current Sense ADC (0-3.3V input only)
```

### Pines Sensor Hall (Retroalimentación)

```cpp
GPIO 33 → Hall Sensor (6 magnets/rev feedback)
```

### Pines Encoder KY-040 (Control Manual)

Configuración Actual Verificada

**Motor Control (VNH5019)**:

- ✅ GPIO 25-27 OK para motor PWM + dirección
- ✅ GPIO 35 OK para corriente (ADC input only)

**Realimentación**:

- ✅ GPIO 33 OK para sensor Hall (6 imanes, RPM feedback)

**Control Manual**:

- ✅ GPIO 18, 19, 23 OK para encoder KY-040

**Periféricos**:

- ✅ GPIO 21, 22 OK para LCD I2C
- ✅ GPIO 13 OK para parada de emergencia
- ✅ GPIO 2 OK para LED indicador

**WiFi**:

- ✅ Opera normalmente (no usa GPIO 25, 26, 27, 33, 18, 19, 23, 21, 22, 13)

### Pines LCD 16x2 (I2C)

```cpp
GPIO 21 → LCD SDA (I2C Data)
GPIO 22 → LCD SCL (I2C Clock)
```

### Pines Seguridad

```cpp
GPIO 13 → Emergency Stop Button
GPIO 2  → Built-in LED Indicatorinput only ADC)
---

## Versiones de ESP32 Soportadas

- ESP32-WROOM-32
- ESP32-WROOM-32D
- ESP32-DevKitC
- Otros módulos con 32 GPIO
