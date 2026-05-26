# ESP32 WiFi Web Server Motor Control

Control de motor DC con VNH5019 Pololu via **Servidor Web WiFi**. Interfaz moderna y responsiva que funciona en cualquier navegador (Android, iOS, Windows, Mac).

---

## 🎯 Características

✅ **Servidor Web Responsivo**

- Interfaz moderna y profesional
- Funciona en cualquier dispositivo (móvil, tablet, PC)
- Sin aplicación requerida

✅ **Control Motor en Tiempo Real**

- Ajustes de 0.2 m/s a 1.9 m/s, con saltos de 0.05 m/s
- Respuesta inmediata

✅ **Gráficos Interactivos**

- Velocidad vs Tiempo (línea)
- Corriente vs Tiempo (línea)

✅ **WebSocket**

- Actualizaciones en tiempo real (~100ms)
- Bajo latencia

✅ **Modos WiFi**

- Punto de acceso integrado (sin WiFi externa)

---

## 🚀 Quick Start (3 pasos)

### 1. Compilar y Subir

```powershell
# Build
pio run

# Upload to ESP32 (connect via USB)
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

**Espera a ver**:

```text
[WiFi] Access Point: ESP32_Motor_Control
[WiFi] AP IP: 192.168.4.1
```

### 2. Conectar desde Móvil

**Android:**

1. Configuración > WiFi
2. Busca red: `ESP32_Motor_Control`
3. Conecta con contraseña: `12345678`

**iOS:**

1. Ajustes > WiFi
2. Busca: `ESP32_Motor_Control`
3. Contraseña: `12345678`

### 3. Abrir en Navegador

```text
http://192.168.4.1
```

**¡Interfaz web lista para controlar!**

---

## 🔧 Configuración

### Cambiar SSID / Contraseña

Edita `include/wifi_config.h`:

```cpp
#define WIFI_SSID "Mi_Red_WiFi"
#define WIFI_PASSWORD "micontraseña"
```

### Conectar a WiFi Existente

1. Edita `include/wifi_config.h`:

    ```cpp
    #define WIFI_SSID "Mi_Router_WiFi"
    #define WIFI_PASSWORD "mi_password_router"
    #define WIFI_MODE AP_MODE  // O STA_MODE para solo cliente 
    ```

2. Upload y abre serial monitor

3. Verás:

    ```cpp
    [WiFi] Connecting to Mi_Router_WiFi...
    [WiFi] WiFi IP: 192.168.1.100
    ```

### Opciones de Modo WiFi

**AP_MODE** (Punto de Acceso - Recomendado para simplicidad):

- ESP32 crea su propia red
- No necesita router externo
- Conecta desde móvil/PC directamente
- Rango: ~30m interior

**STA_MODE** (Cliente WiFi):

- ESP32 se conecta a router existente
- Requiere WiFi 2.4GHz
- Múltiples dispositivos en red local
- Mejor para integración en hogar

**AP_STA_MODE** (Ambos):

- ESP32 es cliente Y punto de acceso simultáneamente
- Máxima flexibilidad

---

## 📡 Conectividad

### Requisitos WiFi

- Frecuencia: 2.4GHz (WiFi 5GHz NO compatible)
- Protocolo: 802.11 b/g/n
- Potencia: Estándar (20dBm)
- Ancho banda: 20MHz

### Rango

- Interior: ~20-30m (depende de obstáculos)
- Exterior: ~50m línea de vista
- Afectado por: paredes, metales, interferencia

### Rendimiento

- Latencia típica: 10-50ms
- Velocidad: WiFi b/g/n (suficiente para control motor)
- Throughput: 1-2 Mbps real (WebSocket es bajo bandwidth)

---

## 🐛 Troubleshooting

### "Cannot connect to ESP32"

1. Verifica ESP32 está powered
2. Revisa puerto serial (pio device list)
3. Reinicia: presiona botón RESET

### "WiFi conecta pero no hay internet"

- Esperado si usas AP_MODE (aislado de internet)
- Normal para control local

### "Latencia alta / Motor responde lento"

- Busca interferencia 2.4GHz (microondas, cordless phones)
- Cambia canal WiFi en router
- Aumenta potencia WiFi (si es configurable)

### "Se desconecta después de minutos"

- Aumenta timeout en `include/wifi_config.h`
- Verifica alimentación (voltaje bajo causa desconexiones)
- Reduce distancia al router

### "No puedo subir web page updates"

- Asegúrate `data/` folder existe
- `pio run --target uploadfs` para subir archivos SPIFFS

---

## 📊 Monitoreo WiFi

En el serial monitor verás:

```text
[WiFi] Starting WiFi...
[WiFi] Mode: AP
[WiFi] SSID: ESP32_Motor_Control
[WiFi] Password: 12345678
[WiFi] AP IP: 192.168.4.1
[HTTP] Web server started on port 80
[WS] WebSocket server started
```

### Debug WiFi Signal

Edita `include/wifi_config.h`:

```cpp
#define DEBUG_WIFI true  // Muestra RSSI cada segundo
```

Serial output:

```text
[WiFi] RSSI: -45 dBm (Excelente)
[WiFi] RSSI: -60 dBm (Bueno)
[WiFi] RSSI: -75 dBm (Aceptable)
[WiFi] RSSI: -85 dBm (Débil)
```

---

### 🔐 Seguridad Básica

Por defecto, ESP32 **NO está encriptado** (open network option possible):

Si necesitas seguridad:

1. Edita `include/wifi_config.h`
2. Configura contraseña fuerte (>8 caracteres)
3. Usa WPA2 (automático si contraseña configurada)
