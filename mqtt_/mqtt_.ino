#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <HardwareSerial.h>
#include <map>

// ── WiFi / MQTT ──────────────────────────────────────────────────────────────
const char* SSID        = "Wi-Fio LabEtronica";
const char* PASS        = "wds2008fio";
const char* MQTT_SERVER = "192.168.25.140";
const int   MQTT_PORT   = 1883;

// ── UART2: RX=GPIO5 ← TX maestro, TX=GPIO4 → RX maestro ────────────────────
#define GW_RX_PIN 5
#define GW_TX_PIN 4
HardwareSerial SerialMaster(2);

// ── Tópicos ──────────────────────────────────────────────────────────────────
#define TOPIC_TOTALES  "totales"
#define TOPIC_CONSUMOS "consumos"

// ── Globales ─────────────────────────────────────────────────────────────────
WiFiClient   espClient;
PubSubClient mqtt(espClient);
String       rxBuf = "";

// ── Mapa de consumos por nodo (MAC → potencia en W) ──────────────────────────
// Refleja exactamente lo que hace comm_GetTotalConsumption() en el Agente de Carga
std::map<String, float> peerPower;

// ── Suma total de consumo de todos los nodos registrados ─────────────────────
float getTotalConsumption() {
  float total = 0.0f;
  for (auto& p : peerPower) {
    total += p.second;
  }
  return total;
}

// ── WiFi ─────────────────────────────────────────────────────────────────────
void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  Serial.print("WiFi");
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
  Serial.printf(" OK — IP: %s\n", WiFi.localIP().toString().c_str());
}

// ── MQTT reconnect (no bloqueante) ───────────────────────────────────────────
void mqtt_reconnect() {
  static unsigned long last = 0;
  if (mqtt.connected() || millis() - last < 5000) return;
  last = millis();
  Serial.print("MQTT...");
  if (mqtt.connect("ESP32_Gateway")) Serial.println(" OK");
  else Serial.printf(" fallo rc=%d\n", mqtt.state());
}

// ── Parser / publisher ────────────────────────────────────────────────────────
void parseAndPublish(const String& line) {
  if (!mqtt.connected()) return;

  // VRMS:tension,corriente   → se guarda hasta recibir CAN_PWR
  // CAN_PWR:potencia         → publica totales con consumo_total acumulado de peers
  // PEER:mac,consumo,prio    → actualiza mapa de peers y publica consumo individual

  static float s_tension   = 0;
  static float s_corriente = 0;
  static float s_potencia  = 0;

  if (line.startsWith("VRMS:")) {
    // "VRMS:220.00,1.500"
    int comma = line.indexOf(',', 5);
    if (comma == -1) return;
    s_tension   = line.substring(5, comma).toFloat();
    s_corriente = line.substring(comma + 1).toFloat();

  } else if (line.startsWith("CAN_PWR:")) {
    // "CAN_PWR:100.0"
    s_potencia = line.substring(8).toFloat();

    // Calcular consumo_total sumando el power de todos los peers registrados
    // (equivalente a comm_GetTotalConsumption() del Agente de Carga)
    float consumo_total = getTotalConsumption();

    char payload[160];
    snprintf(payload, sizeof(payload),
             "{\"potencia\":%.2f,\"corriente\":%.4f,\"tension\":%.2f,\"consumo_total\":%.4f}",
             s_potencia, s_corriente, s_tension, consumo_total);
    mqtt.publish(TOPIC_TOTALES, payload, true);
    Serial.printf("→ totales: %s\n", payload);

  } else if (line.startsWith("PEER:")) {
    // "PEER:AA:BB:CC:DD:EE:FF,1.2345,1"
    String body = line.substring(5);
    int c1 = body.indexOf(',');
    int c2 = body.indexOf(',', c1 + 1);
    if (c1 == -1 || c2 == -1) return;

    String mac       = body.substring(0, c1);
    float  consumo   = body.substring(c1 + 1, c2).toFloat();
    int    prioridad = body.substring(c2 + 1).toInt();

    // Si el nodo reporta consumo 0 significa que está desconectado: eliminarlo del mapa
    // para que no sume al total. Si no existe tampoco, no hacemos nada.
    if (consumo == 0.0f) {
      peerPower.erase(mac);
    } else {
      // Actualizar el mapa con el último valor reportado por este nodo
      peerPower[mac] = consumo;
    }

    char payload[128];
    snprintf(payload, sizeof(payload),
             "{\"nodo\":\"%s\",\"consumo\":%.4f,\"prioridad\":%d}",
             mac.c_str(), consumo, prioridad);
    mqtt.publish(TOPIC_CONSUMOS, payload, true);
    Serial.printf("→ consumos: %s\n", payload);
  }
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  SerialMaster.begin(115200, SERIAL_8N1, GW_RX_PIN, GW_TX_PIN);
  setup_wifi();
  mqtt.setServer(MQTT_SERVER, MQTT_PORT);
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  mqtt_reconnect();
  mqtt.loop();

  while (SerialMaster.available()) {
    char c = SerialMaster.read();
    if (c == '\n') {
      rxBuf.trim();
      if (rxBuf.length()) parseAndPublish(rxBuf);
      rxBuf = "";
    } else if (c != '\r') {
      // Guardia de overflow: si el buffer crece más de lo razonable
      // (una línea válida tiene como máximo ~80 chars) es porque
      // dos mensajes llegaron pegados sin '\n' entre medio.
      // Descartamos el buffer corrupto para no publicar MACs concatenadas.
      if (rxBuf.length() > 96) {
        rxBuf = "";
      } else {
        rxBuf += c;
      }
    }
  }
}
