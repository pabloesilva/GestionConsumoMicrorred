#include <Arduino.h>
#include <driver/adc.h>
#include <esp_now.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <vector>

static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// Estructuras
struct availability_msg_t {
  float availablePower;
};

struct consensus_msg_t {
  float power;
  uint8_t priority;
};

struct PeerData {
  uint8_t mac[6];
  float power;
  uint8_t priority;
  unsigned long lastSeen;
};

static std::vector<PeerData> peers;
WebServer server(80);
const unsigned long WINDOW_MS = 200;
float availablePower = 0.0f;
float totalConsumption = 0.0f;

void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > 10 * WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

void onDataRecv(const uint8_t* mac, const uint8_t* buf, int len) {
  if (len == sizeof(consensus_msg_t)) {
    consensus_msg_t msg;
    memcpy(&msg, buf, len);
    for (auto &p : peers) {
      if (memcmp(p.mac, mac, 6) == 0) {
        p.power = msg.power;
        p.priority = msg.priority;
        p.lastSeen = millis();
        return;
      }
    }
    PeerData np;
    memcpy(np.mac, mac, 6);
    np.power = msg.power;
    np.priority = msg.priority;
    np.lastSeen = millis();
    peers.push_back(np);
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("NodoInformante", "12345678");
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error inicializando ESP-NOW");
    while (true) delay(1000);
  }
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error agregando peer broadcast");
  }
}

void handleRoot() {
  const char* html = R"rawliteral(
<!doctype html><html>
<head>
  <meta charset="utf-8">
  <title>Dashboard Energía</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
  <style>
    body { font-family: sans-serif; padding: 20px; }
    h1 { font-size: 24px; }
    table { border-collapse: collapse; margin-top: 20px; }
    th, td { border: 1px solid black; padding: 5px; }
    #chart-container { width: 100%; max-width: 700px; margin-top: 40px; }
  </style>
</head>
<body>
  <h1>Dashboard Energía</h1>
  <p>Potencia Disponible: <span id="avail">–</span> W</p>
  <p>Consumo Total: <span id="total">–</span> W</p>
  <table id="nodes">
    <tr><th>Dirección MAC</th><th>Consumo(W)</th><th>Prioridad</th><th>Ultimo Mensaje(ms)</th></tr>
  </table>
  <div id="chart-container">
    <canvas id="chart"></canvas>
  </div>

  <script>
    const historyLength = 60;
    const labels = [];
    const datasets = {};
    const totalPowerData = [];

    const ctx = document.getElementById('chart').getContext('2d');
    const chart = new Chart(ctx, {
      type: 'line',
      data: {
        labels: labels,
        datasets: []
      },
      options: {
        responsive: true,
        animation: false,
        scales: {
          x: { title: { display: true, text: 'Tiempo (s)' } },
          y: { title: { display: true, text: 'Potencia (W)' }, min: 0 }
        }
      }
    });

    async function refresh() {
      const resp = await fetch('/data');
      const j = await resp.json();

      const now = new Date().toLocaleTimeString();
      labels.push(now);
      if (labels.length > historyLength) labels.shift();

      document.getElementById('avail').textContent = j.availablePower.toFixed(2);
      
      let totalPower = 0;
      const tbl = document.getElementById('nodes');
      tbl.innerHTML = '<tr><th>Dirección MAC</th><th>Consumo(W)</th><th>Prioridad</th><th>Ultimo Mensaje(ms)</th></tr>';
      j.nodes.forEach(n => {
        totalPower += n.power;
        tbl.innerHTML += `<tr><td>${n.mac}</td><td>${n.power.toFixed(2)}</td><td>${n.priority}</td><td>${n.age}</td></tr>`;

        if (!datasets[n.mac]) {
          const color = '#' + Math.floor(Math.random()*16777215).toString(16).padStart(6, '0');
          datasets[n.mac] = {
            label: n.mac,
            borderColor: color,
            fill: false,
            data: []
          };
          chart.data.datasets.push(datasets[n.mac]);
        }

        datasets[n.mac].data.push(n.power);
        if (datasets[n.mac].data.length > historyLength) datasets[n.mac].data.shift();
      });

      document.getElementById('total').textContent = totalPower.toFixed(2);

      // Total consumption tracking
      if (!datasets['total']) {
        datasets['total'] = {
          label: "Total",
          borderColor: "#000",
          borderWidth: 2,
          fill: false,
          data: []
        };
        chart.data.datasets.push(datasets['total']);
      }
      datasets['total'].data.push(totalPower);
      if (datasets['total'].data.length > historyLength) datasets['total'].data.shift();

      chart.update();
    }

    setInterval(refresh, 1000);
    window.onload = refresh;
  </script>
</body>
</html>
)rawliteral";
  server.send(200, "text/html", html);
}



void handleData() {
  // usamos JsonDocument en lugar de DynamicJsonDocument
  JsonDocument doc;

  doc["availablePower"] = availablePower;

  JsonArray arr = doc["nodes"].to<JsonArray>();  // creamos el arreglo de nodos
  unsigned long now = millis();
  float total = 0;

  for (auto &p : peers) {
    JsonObject o = arr.add<JsonObject>();        // añadimos un objeto por nodo
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
      p.mac[0],p.mac[1],p.mac[2],p.mac[3],p.mac[4],p.mac[5]);
    o["mac"]      = macStr;
    o["power"]    = p.power;
    o["priority"] = p.priority;
    o["age"]      = now - p.lastSeen;
    total += p.power;
  }

  doc["totalConsumption"] = total;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}


void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
}

void setup() {
  Serial.begin(115200);
  setupEspNow();
  setupWebServer();
}

void loop() {
  availablePower = random(1000, 2201);  // entre 1000 y 2200 W
  availability_msg_t a = { availablePower };
  esp_now_send(broadcastAddress, (uint8_t*)&a, sizeof(a));

  delay(WINDOW_MS);
  purgeStalePeers();
  server.handleClient();
  delay(1000 - WINDOW_MS);
}
