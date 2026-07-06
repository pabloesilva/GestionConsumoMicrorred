# Nodo Informante (NI)

Firmware para ESP32 que actúa como puente entre tres sistemas de la microrred: el **bus CAN** hacia el Agente de Gestión (AG), la **red ESP-NOW** de Nodos de Consumo (NC), y un enlace **UART** hacia un Gateway externo de monitoreo/telemetría.

El NI recibe por CAN la potencia instantánea del panel medida por el AG, la combina con una medición propia de tensión RMS para calcular la corriente disponible en la red, y la retransmite por ESP-NOW a los NC — cumpliendo así el rol que el README del NC describe como **Agente de Carga (AC)**. En simultáneo, escucha los broadcasts de consenso de los NC, arma una tabla de peers y reporta el consumo total agregado de vuelta al AG por CAN. Todo el tráfico (paquetes CAN, mensajes de consenso, estado de peers) se vuelca además por UART hacia un Gateway para registro/monitoreo externo. Un display OLED con 3 pantallas navegables por botones permite inspeccionar el estado del nodo sin depender del Gateway.

> Para el detalle del algoritmo de consenso, prioridades y cascada de desconexión que corren en cada NC, ver el README del repositorio del NC.

---

## Arquitectura

```
  Agente de Gestion (AG)                                   Gateway (monitoreo)
        |                                                        ^
        | CAN 500kbps (id 0x610: V,I panel)                      | UART 115200
        |                                                        | (VRMS / CAN_PWR / PEER)
        v                                                        |
  +----------------------------------------------------------------+
  |                      Nodo Informante (NI)                       |
  +----------------------------------+------------------------------+
                                     |
        CAN 0x620 (consumo total) <-+-> ESP-NOW broadcast
                                          - TX: availability_msg_t (corriente disponible)
                                          - RX: consensus_msg_t (consumo + prioridad) de cada NC
        (vuelve al AG)                        |
                                               v
                                    Nodos de Consumo (NC) x N
```

---

## Hardware

| Componente | Detalle |
|-----------|---------|
| MCU | ESP32 (dual-core 240MHz) |
| Controlador CAN | MCP2515 — SPI (VSPI por defecto: SCK=18, MISO=19, MOSI=23), CS=GPIO5, INT=GPIO4 |
| Sensor de tensión | ADC1_CH5 — GPIO33 |
| Display | OLED SSD1306 128x64 — I2C (SDA=21, SCL=22) |
| Botones | 3x pulsador a GND (PREV, NEXT, HOME) con `INPUT_PULLUP` |
| UART Gateway | UART0 remapeado (TX=GPIO1, RX=GPIO3) — comparte los pines del USB-serial, por eso el `Serial` de depuración queda deshabilitado en `setup()` |

### Pinout

```
GPIO33  -> Sensor de tension (entrada analogica)
GPIO5   -> CAN CS
GPIO4   -> CAN INT
GPIO18  -> CAN SCK  (VSPI)
GPIO19  -> CAN MISO (VSPI)
GPIO23  -> CAN MOSI (VSPI)
GPIO21  -> OLED SDA
GPIO22  -> OLED SCL
GPIO14  -> Boton PREV
GPIO27  -> Boton NEXT
GPIO26  -> Boton HOME
GPIO1   -> UART TX (hacia Gateway)
GPIO3   -> UART RX (desde Gateway)
```

---

## Medición de tensión (Vrms)

Al arranque, antes de iniciar CAN o ESP-NOW, el nodo calibra el offset de su sensor de tensión con el mismo método que usa el NC para su ACS712: 10.000 muestras a 50kHz (10 períodos de la red de 50Hz), promediadas para obtener `voltageOffset`.

En cada ciclo, con el buffer lleno:

```
1. Adquirir 10.000 muestras del ADC
2. Convertir cada muestra a voltios
3. Restar el offset calibrado -> V_ac(i)
4. Vrms = sqrt( Σ V_ac(i)² / N )
5. Vrms = Vrms * 998   // escala para llevarlo a la unidad real (pendiente de revisión, ver comentario en el código)
```

A diferencia del NC, esta medición **no aplica ventana de histéresis** sobre el resultado. `Vrms` se usa junto con la potencia recibida del AG por CAN para derivar la corriente disponible (`I = P / Vrms`), y solo se considera válida si `Vrms >= V_MIN` (30V).

---

## Comunicación CAN (con el Agente de Gestión)

Bus a 500kbps sobre MCP2515 (oscilador de 8MHz, con fallback a 16MHz en el reintento de reset).

### Recepción — potencia del panel (id `0x610`)

El AG envía tensión y corriente del panel como enteros de 16 bits little-endian, escalados 0..4095 sobre el rango `0..V_MAX` / `0..I_MAX`:

```
data[0..1] = V_panel  (uint16, 0..4095 -> 0..V_MAX)
data[2..3] = I_panel  (uint16, 0..4095 -> 0..I_MAX)
```

El NI reconstruye ambos valores, los clampea a `[0, V_MAX]` / `[0, I_MAX]`, descarta como cero cualquier valor menor a 0.01 (ruido), y calcula `availablePower = V_panel * I_panel`. Si no llega un frame válido durante `DATA_EXPIRY_MS` (10s), `panelDataValid` pasa a `false` y el resto del sistema deja de considerar ese dato utilizable.

### Envío — consumo total de la red (id `0x620`)

Cada `canInterval` (5s), el NI informa al AG el consumo total agregado de los NC activos (suma de la tabla de peers), codificado como entero de 16 bits little-endian con 2 decimales de precisión:

```
data[0..1] = (uint16_t)(totalConsumption * 100)
```

### Recuperación ante fallos

- Si la inicialización del MCP2515 falla, se reintenta cada `CAN_RETRY_INTERVAL_MS` (5s) indefinidamente.
- Si se acumulan `CAN_RESET_THRESHOLD` (5) fallos consecutivos de envío, o no hay un envío exitoso en `CAN_FORCE_RESET_MS` (30s) aunque los fallos no lleguen al umbral, el módulo se marca fuera de servicio y se fuerza un reset completo del MCP2515.
- Cada intento de envío tiene hasta `MAX_CAN_RETRIES` (3) reintentos, leyendo y descartando mensajes pendientes del controlador entre intento e intento para no dejarlo trabado.

---

## Comunicación ESP-NOW (con los NC)

### Disponibilidad (TX — hacia los NC)

```cpp
struct availability_msg_t {
    float availableCurrent;   // A, = availablePower / Vrms
};
```

Se retransmite por broadcast cada `availabilityInterval` (= `WINDOW_MS`, 500ms). Vale 0 si `Vrms < V_MIN`, si no hay potencia disponible, o si el dato del AG expiró (`panelDataValid == false`).

### Consenso (RX — desde los NC)

```cpp
struct consensus_msg_t {
    float power;        // en Amps pese al nombre del campo -- consumo del NC (Arms)
    uint8_t priority;    // prioridad configurada por DIP switch en el NC
};
```

Cada mensaje recibido actualiza la tabla de peers (hasta 8, dirección MAC como clave) y se reenvía de inmediato por UART al Gateway como una línea `PEER:...` (no espera al ciclo periódico).

### Tabla de peers y "zombies"

```cpp
struct PeerData {
    uint8_t mac[6];
    float power;              // ultimo consumo reportado (A)
    uint8_t priority;
    unsigned long lastSeen;
    unsigned long secondLastSeen;
};
```

Un peer sin mensajes por más de `4 * WINDOW_MS` (2s) se elimina de la tabla activa (deja de sumar al consumo total y desaparece de la pantalla de nodos), y pasa a una lista de **zombies** que conserva su última prioridad conocida. Mientras esté en esa lista, el NI le informa al Gateway por UART `PEER:<mac>,0.0000,<priority>` en cada ciclo, para que el sistema de monitoreo lo vea caer a cero sin perder de vista qué prioridad tenía. Si el peer vuelve a transmitir, se lo saca de la lista de zombies.

> A diferencia del NC, este mecanismo es solo para telemetría hacia el Gateway — el `power=0` de un zombie nunca se inyecta en `comm_GetTotalConsumption()` ni en el mensaje CAN 0x620.

---

## Comunicación UART (con el Gateway)

UART0 remapeada a los pines TX=1/RX=3 a 115200 baud (por eso `Serial` de debug está deshabilitado — comparte el mismo periférico físico). Cada `uartSendInterval` (500ms) el NI emite:

```
VRMS:<Vrms>,<availableCurrent>      // Vrms con 2 decimales, corriente disponible con 3
CAN_PWR:<availablePower>            // potencia del panel (W, 1 decimal), 0 si invalida
PEER:<mac>,0.0000,<priority>        // uno por cada peer en estado "zombie"
```

A eso se suma, de forma asincrónica apenas llega un broadcast de consenso:

```
PEER:<mac>,<power>,<priority>       // power con 4 decimales
```

---

## Display

OLED de 3 pantallas navegadas manualmente con los botones PREV/NEXT (sin rotación automática por tiempo, a diferencia del NC):

**Pantalla 0 — Principal:** potencia del panel (o `N/A` si el dato del AG expiró), Vrms medido, corriente máxima disponible (`Imax = P/Vrms`, o `--` si inválido) y consumo total de la red.

**Pantalla 1 — Nodos:** tabla paginada de peers activos (5 por página: MAC parcial, consumo en A, antigüedad del último mensaje en ms, prioridad).

**Pantalla 2 — Estado CAN:** `OK` / `FAILED` / `INIT FAIL`, cantidad de fallos consecutivos, intervalo del último envío exitoso.

Cada pantalla solo se redibuja cuando alguno de sus valores cambia (o se fuerza un redraw al cambiar de pantalla), para minimizar el tiempo bloqueado en I2C.

Botón HOME: mantenido presionado ≥ `longPressMs` (3s) fuerza un `ESP.restart()` — útil como recuperación manual sin necesidad de recortar alimentación o reflashear.

---

## Dependencias

```ini
; platformio.ini
[env:esp32doit-devkit-v1]
platform = espressif32
board = esp32doit-devkit-v1
framework = arduino
monitor_speed = 115200

lib_deps =
    adafruit/Adafruit SSD1306@^2.5.7
    adafruit/Adafruit GFX Library@^1.11.9
    autowp/autowp-mcp2515@^1.0.1
```

---

## Parámetros de configuración

```cpp
// CAN
const int CAN_CS_PIN                 = 5;
const int CAN_INT_PIN                = 4;
const int MAX_CAN_RETRIES            = 3;
const int CAN_RESET_THRESHOLD        = 5;     // fallos consecutivos antes de forzar reset
const unsigned long CAN_RETRY_INTERVAL_MS = 5000;
const unsigned long CAN_FORCE_RESET_MS    = 30000; // sin exito en este tiempo -> reset forzado
const unsigned long canInterval      = 5000;  // periodo de envio del consumo total

// Panel / AG
const float V_MAX  = 100.0f;
const float I_MAX  = 50.0f;
const float V_MIN  = 30.0f;    // Vrms minimo para considerar la medicion valida
const unsigned long DATA_EXPIRY_MS = 10000; // sin frames del AG -> panelDataValid = false

// Peers / ESP-NOW
const unsigned long WINDOW_MS = 500; // periodo de broadcast de disponibilidad; purge = 4xWINDOW_MS
const unsigned long availabilityInterval = WINDOW_MS;

// ADC / Vrms
const int sensorPin  = 33;
const int Fs         = 50000;
const int lineFreq   = 50;
const int periodsToCapture = 10;

// UART Gateway
const int GW_TX_PIN = 1;
const int GW_RX_PIN = 3;
const unsigned long uartSendInterval = 500;

// Botones
const int BTN_PREV_PIN = 14;
const int BTN_NEXT_PIN = 27;
const int BTN_HOME_PIN = 26;
const unsigned long debounceMs  = 15;
const unsigned long longPressMs = 3000;
```
