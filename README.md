# Gateway MQTT

Sketch para ESP32 (NodeMCU-32S / ESP32 Dev Module) que recibe por UART la telemetría del **Agente de Carga (AC)** y la republica como JSON en un broker MQTT. Es el último eslabón de la cadena `AG → AC → Gateway → MQTT`: no participa del CAN ni del ESP-NOW, solo consume el texto plano que el AC ya emite por su UART hacia el Gateway.

> Se comparte como `.cpp` pero es un sketch de Arduino (`.ino`): usa `setup()`/`loop()` y se compila tal cual con el core de ESP32.

---

## Hardware / conexión

| Componente | Detalle |
|-----------|---------|
| UART2 (`SerialMaster`) | RX=GPIO5 (recibe del TX del AC), TX=GPIO4 (no usado por el AC, full-duplex disponible) — 115200 baud |
| WiFi | STA, credenciales hardcodeadas en el sketch (`SSID`/`PASS`) |
| MQTT | `PubSubClient` contra `192.168.25.116:1883`, client id `"ESP32_Gateway"` |

Corre igual en cualquier ESP32 clásico (NodeMCU-32S, DevKit, etc.): no usa nada específico del S3 (ni USB-CDC nativo, ni pines reservados de flash/PSRAM). GPIO4 y GPIO5 son pines de propósito general disponibles en cualquiera de estas placas — al migrar de placa solo hay que revisar el cableado físico hacia el AC, no el código.

---

## Framing UART

Igual que en el AC: líneas de texto terminadas en `\n` (el `\r` se descarta). El buffer se arma carácter a carácter en `rxBuf` y se procesa completo al llegar el `\n`.

Guarda de overflow: si `rxBuf` supera 96 caracteres sin encontrar un `\n`, se descarta entero. Evita que dos líneas pegadas por un `\n` perdido terminen publicando una MAC concatenada o basura al broker.

---

## Parseo (`parseAndPublish`)

Reconoce los tres prefijos que emite el AC (ver su propio README para el detalle de cuándo y por qué los envía):

| Prefijo | Acción |
|---------|--------|
| `VRMS:<v>,<i>` | Guarda tensión y corriente disponible en estáticas (`s_tension`, `s_corriente`); no publica nada por sí solo |
| `CAN_PWR:<p>` | Guarda `s_potencia` y dispara la publicación en `totales` (ver abajo) |
| `PEER:<mac>,<consumo>,<prioridad>` | Actualiza el mapa local de peers y publica en `consumos` |

### Mapa de peers y consumo total

```cpp
std::map<String, float> peerPower;   // MAC -> potencia (W)
```

El Gateway no recibe el consumo total agregado del AC (ese valor solo viaja por CAN hacia el AG) — lo recalcula localmente sumando `peerPower`, replicando `comm_GetTotalConsumption()` del AC.

Para mantener el mapa sincronizado con la tabla de peers activos del AC, interpreta **cualquier** `PEER` con `consumo == 0.0f` como una baja: borra esa MAC del mapa en lugar de guardar un 0. Esto depende de que el AC solo emita `0.0000` exacto para peers "zombie" (nunca como lectura real intermedia), ya que la comparación es de igualdad flotante estricta.

### Publicación `totales`

Disparada por cada línea `CAN_PWR` (o sea, al ritmo del `uartSendInterval` del AC, 500ms):

```json
{"potencia":100.00,"corriente":1.5000,"tension":220.00,"consumo_total":3.2500}
```

### Publicación `consumos`

Disparada por cada línea `PEER` (tanto actualizaciones reales como ceros de peers zombie):

```json
{"nodo":"AA:BB:CC:DD:EE:FF","consumo":1.2345,"prioridad":1}
```

Ambos tópicos se publican con el flag `retain = true`, para que un cliente que se suscribe recién arrancado vea el último estado sin esperar al próximo ciclo.

---

## WiFi / MQTT

- `setup_wifi()` bloquea en `setup()` hasta conectar (reintenta cada 500ms).
- `mqtt_reconnect()` es no bloqueante: solo intenta reconectar si pasaron ≥5s desde el último intento fallido, se llama en cada vuelta de `loop()`.
- No hay usuario/contraseña ni TLS configurados en la conexión MQTT.

---

## Dependencias

```cpp
#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <map>
```

---

## Notas / puntos a revisar

- El SSID, password y IP del broker están en texto plano en el código fuente.
- La detección de "nodo desconectado" depende de que el AC mande `0.0000` literal — si en algún momento se agrega ruido o redondeo distinto en ese campo, el `erase()` dejaría de dispararse y el peer quedaría fantasma en el total.
- El tamaño máximo de línea (96 caracteres) es un límite empírico basado en el largo actual de los mensajes del AC; si se agregan campos a `PEER:` o `VRMS:` hay que revisar que sigan entrando en ese margen.
