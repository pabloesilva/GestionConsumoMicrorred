#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <SPI.h>
#include <MCP2515.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>

// --- ESPNOW broadcast address ---
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// --- Data structures ---
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
const unsigned long WINDOW_MS = 500;

// --- CAN (MCP2515) ---
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canMsg;

// --- OLED 0.96" ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- CAN (MCP2515) ---
#define MAX_CAN_RETRIES 3

// --- ESPNOW callback (unchanged) ---
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

// --- Purge inactive peers ---
void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = (int)peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > 10 * WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

// --- Setup ESPNOW ---
void setupEspNow() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP("NodoInformante","12345678");
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
  esp_now_add_peer(&peerInfo);
}

// --- Function to display text on OLED ---
void displayMessage(const char* msg, bool clear = true) {
  if (clear) display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(msg);
  display.display();
}

void setup() {
  Serial.begin(115200);

  // OLED init
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true);
  }
  display.clearDisplay();
  display.display();

  // CAN init
  SPI.begin(); // uses default pins: MOSI=23,MISO=19,SCK=18
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    displayMessage("CAN Bitrate Error!");
    while(1);
  }
  mcp2515.setNormalMode();

  // ESPNOW init
  setupEspNow();
}

void loop() {
  // 1) simulate available power
  float availablePower = random(0, 2201);

  // 2) broadcast availability
  availability_msg_t a = { availablePower };
  esp_now_send(broadcastAddress, (uint8_t*)&a, sizeof(a));

  // 3) purge & compute totals
  purgeStalePeers();
  float totalConsumption = 0;
  unsigned long now = millis();
  int nodeCount = peers.size();
  unsigned long minAge = 0;
  for (auto &p : peers) {
    totalConsumption += p.power;
    unsigned long age = now - p.lastSeen;
    if (minAge == 0 || age < minAge) minAge = age;
  }
  if (nodeCount == 0) minAge = 0;

  // 4) update OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("Pot. Disp: %.1f W", availablePower);
  display.setCursor(0, 10);
  display.printf("Consumo : %.1f W", totalConsumption);
  display.setCursor(0, 20);
  display.printf("Nodos: %d", nodeCount);
  display.setCursor(0, 30);
  display.printf("Edad(ms): %lu", minAge);
  display.display();
  delay(500); // Wait a bit to show main info

  // 5) send total consumption via CAN (ID 0x200)
  uint32_t totalConsumption_as_int = static_cast<uint32_t>(totalConsumption * 100); // Multiply by 100 to keep two decimal places
  
  canMsg.can_id = 0x603;
  canMsg.can_dlc = 4;
  canMsg.data[0] = (totalConsumption_as_int >> 24) & 0xFF;
  canMsg.data[1] = (totalConsumption_as_int >> 16) & 0xFF;
  canMsg.data[2] = (totalConsumption_as_int >> 8) & 0xFF;
  canMsg.data[3] = totalConsumption_as_int & 0xFF;

  bool messageSent = false;
  int retries = 0;

  while (!messageSent && retries < MAX_CAN_RETRIES) {
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      messageSent = true;
      // You can add a success message here if you want
    } else {
      displayMessage("Error CAN: Reintentando...", false);
      delay(500); // Pause to show error message
      retries++;
    }
  }

  if (!messageSent) {
    displayMessage("Error CAN: Fallo de envio!");
    delay(2000); // Show final error message for 2 seconds
  }

  delay(1000);
}