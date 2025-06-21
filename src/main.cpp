#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>

// direccion broadcast para ESP‑NOW
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// estructura de mensaje de consenso
typedef struct {
  float power;
} consensus_msg_t;

// almacenamiento de potencias recibidas
static std::vector<uint32_t> peerPowers;

// callback de recepcion ESP‑NOW
void onDataRecv(const uint8_t* mac, const uint8_t* buf, int len) {
  if (len == sizeof(consensus_msg_t)) {
    consensus_msg_t msg;
    memcpy(&msg, buf, len);
    // escalamos a int para evitar push de float
    uint32_t val = (uint32_t)(msg.power * 1000.0f);
    peerPowers.push_back(val);
  }
}

void setup() {
  Serial.begin(115200);

  // configurar WiFi en modo estación (necesario para ESP‑NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // iniciar ESP‑NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("error inicializando esp‑now");
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
  // 1) generar potencia aleatoria (0.00 a 100.00 W)
  float localPower = random(0, 10001) / 100.0f;

  // 2) enviar propia medicion
  consensus_msg_t msg = { localPower };
  esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));

  // 3) ventana de recepcion (espera 100 ms)
  delay(100);

  // 4) calcular consenso medio
  float sumP = localPower;
  int count = 1;
  for (uint32_t v : peerPowers) {
    sumP += (float)v / 1000.0f;
    count++;
  }
  peerPowers.clear();
  float consensus = sumP;

  // 5) imprimir resultados
  Serial.printf("local=%.2f W  nodos=%d  consenso=%.2f W\n",
                localPower, count, consensus);

  // 6) esperar antes del siguiente ciclo
  delay(1000);
}
