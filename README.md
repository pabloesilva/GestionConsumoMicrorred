# Nodo de Consumo (NC)

Firmware para ESP32 que gestiona cargas eléctricas de forma autónoma dentro de una microrred solar. Cada nodo mide su propio consumo de corriente, se comunica con los demás nodos por ESP-NOW, y decide de forma independiente si conectar o desconectar su relé según la energía disponible y un esquema de prioridades configurable por hardware.

Los NC dependen del **Agente de Carga (AC)** para recibir el valor de corriente disponible en la red — sin ese dato no pueden evaluar si deben desconectarse. La descripción completa del AC y la arquitectura general del sistema se encuentra en su propio repositorio.

---

## Hardware

| Componente | Detalle |
|-----------|---------|
| MCU | ESP32 (dual-core 240MHz) |
| Sensor de corriente | ACS712 — GPIO35 (ADC1_CH7) |
| Display | OLED SSD1306 128×32 — I2C (SDA=21, SCL=22) |
| Relé | GPIO25 |
| Configuración de prioridad | DIP switch 2 bits — GPIO26 (MSB), GPIO14 (LSB) |

### Pinout

```
GPIO35  ←  ACS712 (salida analógica)
GPIO25  →  Relé
GPIO26  ←  DIP switch bit 1 (MSB)
GPIO14  ←  DIP switch bit 0 (LSB)
GPIO21  ↔  OLED SDA
GPIO22  ↔  OLED SCL
```

---

## Configuración de prioridad

La prioridad se configura físicamente con un DIP switch de 2 posiciones antes del encendido. No cambia en tiempo de ejecución.

| DIP SW1 | DIP SW0 | Prioridad | Comportamiento |
|:-------:|:-------:|-----------|----------------|
| OFF | OFF | **P0 — Crítica** | Nunca se desconecta |
| OFF | ON  | **P1 — Alta**    | Corta último |
| ON  | OFF | **P2 — Media**   | Corta segundo |
| ON  | ON  | **P3 — Baja**    | Corta primero |

Los switches son activos en bajo: `OFF = HIGH`, `ON = LOW`.

---

## Medición de corriente

### Calibración de offset

En el arranque, antes de iniciar ESP-NOW o cualquier lógica de red, el nodo realiza una calibración del offset del sensor. El ACS712 tiene una tensión de salida de referencia de ~2.5V cuando la corriente es cero, que varía levemente por tolerancias de fabricación y temperatura.

El proceso toma 10.000 muestras a 50kHz (10 períodos completos de la red de 50Hz) y calcula el promedio — ese valor queda como `voltageOffset` y se resta en cada medición posterior. El display muestra una barra de progreso durante este proceso.

```
Fs          = 50.000 Hz
Períodos    = 10
Muestras    = 10.000
Resolución  = 12 bits
```

### Cálculo de corriente RMS

Una vez calibrado el offset, el cálculo de corriente eficaz sigue el método discreto estándar:

```
1. Adquirir 10.000 muestras del ADC
2. Convertir cada muestra a voltios
3. Restar el offset calibrado → V_ac(i)
4. Calcular Vrms = sqrt( Σ V_ac(i)² / N )
5. Aplicar ventana de histéresis: si Vrms < 0.018V → Vrms = 0
6. Irms = Vrms / sensibilidad_sensor   (sensibilidad = 0.072 V/A)
```

La ventana de histéresis en el paso 5 elimina el ruido inherente del sensor cuando no circula corriente real, evitando que el nodo reporte consumos espurios de fracciones de ampere.

---

## Comunicación ESP-NOW

Cada NC participa en dos tipos de intercambio de mensajes:

### Mensajes recibidos del Agente de Carga

```cpp
typedef struct {
    float availableCurrent;   // corriente disponible en la red (Arms)
} availability_msg_t;
```

Este mensaje llega de forma periódica desde el AC vía broadcast ESP-NOW. El nodo lo almacena en `msg_disp` y lo usa como referencia para evaluar si debe desconectarse. Si el AC deja de emitir, el último valor recibido se mantiene hasta que llegue uno nuevo.

### Mensajes de consenso entre NC

```cpp
typedef struct {
    float current;      // consumo propio medido (Arms)
    uint8_t priority;   // prioridad configurada por DIP switch
} consensus_msg_t;
```

Luego de cada medición, el NC hace broadcast de su propio consumo y prioridad hacia todos los demás nodos. Simultáneamente recibe los broadcasts de sus peers y actualiza su tabla local.

### Tabla de peers

Cada nodo mantiene un vector de hasta 8 peers con la siguiente información:

```cpp
struct PeerData {
    uint8_t mac[6];         // identificador único del peer
    float current;          // último consumo reportado
    uint8_t priority;       // prioridad del peer
    unsigned long lastSeen; // timestamp del último mensaje recibido
};
```

Los peers que no envían mensajes por más de 5 segundos (`WINDOW_MS`) son eliminados automáticamente de la tabla por `purgeStalePeers()`. Este mecanismo es la base del sistema de desconexión en cascada — ver sección siguiente.

---

## Lógica de desconexión y reconexión

### Evaluación de sobrecarga

Después de la ventana de consenso (500ms), cada nodo calcula el consumo total de la red sumando su propio consumo más el de todos sus peers activos:

```
totalCurrent = consumo_propio + Σ consumo_peers_activos
```

Si `totalCurrent > availableCurrent`, la red está en sobrecarga y algún nodo debe desconectarse.

### Selección del nodo a desconectar

Cada nodo corre la misma lógica de forma independiente para determinar si le toca desconectarse:

1. Busca la prioridad más alta (número mayor = menos crítico) entre todos los peers visibles, incluyéndose a sí mismo.
2. Solo actúa si su propia prioridad coincide con ese máximo — es decir, si es el menos crítico de los activos.
3. En caso de empate de prioridad, desconecta el nodo de **mayor consumo** (libera más energía de una vez y resuelve el sistema más rápido).
4. Los nodos P0 nunca evalúan desconexión, independientemente del estado de la red.

### Desconexión en cascada

El mecanismo de cascada permite que, si desconectar un nodo no es suficiente para resolver la sobrecarga, el siguiente en prioridad también se desconecte automáticamente.

El principio es simple: **cuando un nodo desconecta su relé, deja de enviar mensajes ESP-NOW**. Tras 5 segundos de silencio, los demás nodos lo eliminan de su tabla de peers. En el siguiente ciclo de evaluación, el `totalCurrent` se recalcula sin ese nodo. Si sigue habiendo sobrecarga, el nodo ahora con la prioridad más alta entre los restantes evalúa su desconexión, y así sucesivamente.

> ⚠️ El silencio es la señal de desconexión. Enviar un mensaje explícito con `current = 0.0` cuando el relé está apagado rompe este mecanismo: los peers seguirían viendo al nodo en su tabla con su prioridad original, impidiendo que el siguiente en prioridad evalúe correctamente.

### Reconexión

Mientras el relé está apagado, el nodo evalúa cada 500ms si puede reconectarse:

```
(availableCurrent - totalCurrent_estimado) > 0.05A  →  reconectar
```

El `totalCurrent_estimado` se calcula como el último consumo conocido del propio nodo incrementado un 5% (margen de seguridad), más el consumo de los peers que todavía sigan activos en la tabla. El umbral de 0.05A actúa como histéresis para evitar que el relé oscile en el límite exacto de disponibilidad.

---

## Máquina de estados

El firmware implementa una FSM no bloqueante en el `loop()` principal, lo que permite que el display se actualice a 20fps independientemente del estado del ADC o la red:

```
         arranque / relé reconectado
                  │
                  ▼
        ┌─────────────────┐
        │  ST_FIRST_MEASURE│  espera 1s (estabilización de la carga)
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │  ST_ADC_RUNNING  │  adquiere 10.000 muestras por DMA
        └────────┬────────┘
                 │  buffer lleno → calcula RMS → broadcast consenso
                 ▼
        ┌─────────────────┐
        │ST_CONSENSUS_WAIT │  espera 500ms para recibir peers
        └────────┬────────┘
                 │
                 ▼
        ┌─────────────────┐
        │  ST_EVALUATING   │  purga peers → calcula total → evalúa
        └────────┬────────┘
                 │                        │
          no desconecta                desconecta
                 │                        │
                 ▼                        ▼
        (vuelve a ST_ADC_RUNNING)  ┌─────────────────┐
                                   │  ST_RELE_OFF     │  evalúa reconexión
                                   └────────┬────────┘  cada 500ms
                                            │
                                     hay margen > 0.05A
                                            │
                                            ▼
                                   (vuelve a ST_FIRST_MEASURE)
```

---

## Display

El OLED alterna automáticamente entre tres modos cada 4 segundos:

**Modo 0 — Disponibilidad:**
Barra segmentada que representa la corriente disponible en la red (escala 0–10A, 12 divisiones). Cuando la disponibilidad es cero, la barra desaparece y aparece el texto `! NO DISPONIBLE !` parpadeando cada 1.5 segundos. El valor numérico exacto se muestra debajo de la barra.

**Modo 1 — Consumo local:**
Corriente medida por este nodo en tiempo real, en fuente grande centrada.

**Modo 2 — Prioridad:**
Prioridad configurada (P0–P3) en fuente grande, con descripción textual debajo (`CARGA CRITICA`, `CARGA NC - ALTA`, etc.).

> El display se actualiza siempre desde el `loop()` principal, nunca desde callbacks ESP-NOW. El callback solo modifica variables de estado (`displayLedsOn`, `displayPulseActive`); el redraw real ocurre en `displayUpdate()`. Esto evita conflictos entre I2C (bloqueante) y la tarea WiFi del FreeRTOS donde corren los callbacks.

---

## Dependencias

```ini
; platformio.ini
[env:esp32]
platform = espressif32
board = esp32dev
framework = arduino

lib_deps =
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
```

---

## Parámetros de configuración

```cpp
const unsigned long WINDOW_MS      = 5000;   // ms — timeout para purgar peer inactivo
const float         sensibility    = 0.072f; // V/A — sensibilidad del ACS712
const float         lineFreq       = 50;     // Hz — frecuencia de red
const int           Fs             = 50000;  // Hz — frecuencia de muestreo ADC
const int           periodsToCapture = 10;   // períodos capturados por medición
const float         displayMaxA    = 10.0f;  // A  — escala máxima del display
const int           displayDivisions = 12;   // divisiones de la barra
const int           MAX_PEERS      = 8;      // máximo de nodos NC en la red
```
