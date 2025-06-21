#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>

// direccion broadcast para ESP‑NOW
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// estructura de mensaje de consenso: potencia + prioridad
typedef struct {
  float power;       // consumo o generación en W
  uint8_t priority;  // 0 = más alta, 255 = más baja
} consensus_msg_t;

// datos de cada peer, indexados por MAC
struct PeerData {
  uint8_t mac[6];
  float power;
  uint8_t priority;
  unsigned long lastSeen;
};

static std::vector<PeerData> peers;

// ventana de recepción en milisegundos
const unsigned long WINDOW_MS = 200;

// callback de recepción ESP-NOW
void onDataRecv(const uint8_t* mac, const uint8_t* buf, int len) {
  if (len != sizeof(consensus_msg_t)) return;
  consensus_msg_t msg;
  memcpy(&msg, buf, len);

  // buscar peer existente
  for (auto &p : peers) {
    if (memcmp(p.mac, mac, 6) == 0) {
      p.power    = msg.power;
      p.priority = msg.priority;
      p.lastSeen = millis();
      return;
    }
  }
  // si no existe, agregar nuevo
  PeerData np;
  memcpy(np.mac, mac, 6);
  np.power    = msg.power;
  np.priority = msg.priority;
  np.lastSeen = millis();
  peers.push_back(np);
}

// purgar peers que no hayan enviado en más de WINDOW_MS
void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // configurar WiFi en modo estación (necesario para ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // iniciar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("error inicializando esp-now");
    while (true) { delay(1000); }
  }
  esp_now_register_recv_cb(onDataRecv);

  // agregar peer broadcast
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("error agregando peer broadcast");
  }
}

void loop() {
  // 1) generar potencia local (simulada)
  float localPower = random(0, 10001) / 100.0f;  // 0.00 a 100.00 W
  uint8_t myPriority = 128;                     // ejemplo: prioridad media

  // 2) enviar propio mensaje
  consensus_msg_t msg = { localPower, myPriority };
  esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));

  // 3) esperar ventana para recibir de todos
  delay(WINDOW_MS);

  // 4) purgar peers inactivos
  purgeStalePeers();

  // 5) calcular potencia total (suma de todos los nodos)
  float totalPower = localPower;
  for (auto &p : peers) {
    totalPower += p.power;
  }

  // 6) mostrar estado
  Serial.printf(
    "nodosActivos: %d  consumoTotal: %.2f W\n", 
    peers.size() + 1,     // +1 = este nodo
    totalPower
  );

  // 7) lógica futura: usar p.priority de cada peer para conectar/desconectar cargas

  // 8) esperar antes del próximo ciclo
  delay(1000);
}
