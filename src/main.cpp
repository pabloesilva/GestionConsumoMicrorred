#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <SPI.h>
#include <MCP2515.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <cstring>

static int canConsecFailures = 0;
const int CAN_RESET_THRESHOLD = 5;

static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

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
const unsigned long WINDOW_MS = 600;

static unsigned long lastMessageTime = 0;
static unsigned long secondLastMessageTime = 0;

// Nuevas variables para el intervalo de CAN
static unsigned long lastCanSentTime = 0;
static unsigned long secondLastCanSentTime = 0;

#define CAN_CS_PIN 5
#define CAN_INT_PIN 4
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canMsg;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MAX_CAN_RETRIES 3

const int buttonPin = 27;
volatile bool screenToggleRequested = false;
int screenIndex = 0; // 0 = main consumption screen, 1 = CAN status screen

unsigned long lastButtonChange = 0;
bool lastButtonState = HIGH;
const unsigned long debounceMs = 50;
const unsigned long longPressMs = 3000;
unsigned long buttonPressedSince = 0;

bool canInitOk = false;
unsigned long lastCanSendMillis = 0;

unsigned long lastCanAttemptMillis = 0;
const unsigned long canInterval = 1000;
unsigned long lastAvailabilitySend = 0;
const unsigned long availabilityInterval = WINDOW_MS;

// Variable de estado para controlar la actualización del display
bool displayNeedsUpdate = true; // Se inicializa en true para la primera actualización

// Flag para forzar redraw (usado cuando cambio de pantalla con el botón)
volatile bool forceDisplayRedraw = false;

void updateDisplay();

void macToStr(const uint8_t mac[6], char out[18]) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void displayMessage(const char* msg, bool clear = true) {
  if (clear) display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(msg);
  display.display();
}

// === Pantalla principal tal como la enviaste, ahora con 'force' seguro ===
void showMainScreen(float availablePower, float totalConsumption, int nodeCount, unsigned long messageInterval, const char* lastNodeMac, bool force = false) {
  static float lastTotalConsumption = -1.0;
  static int lastNodeCount = -1;
  static unsigned long lastMessageInterval = 0;
  static char lastNodeMacPrinted[18] = "-";

  // seguridad: comparar macs sólo si es válido, y evitar strcmp(NULL,...)
  bool sameMac = false;
  if (lastNodeMac) {
    sameMac = (strcmp(lastNodeMac, lastNodeMacPrinted) == 0);
  } else {
    sameMac = (strcmp(lastNodeMacPrinted, "-") == 0);
  }

  if (!force) {
    if (abs(totalConsumption - lastTotalConsumption) < 0.1 && nodeCount == lastNodeCount && messageInterval == lastMessageInterval && sameMac) {
      return;
    }
  }

  lastTotalConsumption = totalConsumption;
  lastNodeCount = nodeCount;
  lastMessageInterval = messageInterval;
  if (lastNodeMac) {
    // asegurar terminación nula
    strncpy(lastNodeMacPrinted, lastNodeMac, sizeof(lastNodeMacPrinted)-1);
    lastNodeMacPrinted[sizeof(lastNodeMacPrinted)-1] = '\0';
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("Pot. Disp: %.1f W", availablePower);

  display.setCursor(0, 10);
  display.printf("Consumo : %.1f W", totalConsumption);

  display.setCursor(0, 20);
  display.printf("Nodos  : %d", nodeCount);

  display.setCursor(0, 30);
  display.printf("Ult. msj.: %lums", messageInterval);

  display.setCursor(0, 40);
  if (lastNodeMac && nodeCount > 0) {
    display.printf("%s", lastNodeMac);
  } else {
    display.printf("Ult nodo: -");
  }
  display.display();
}

// === Pantalla CAN tal como la enviaste, con 'force' ===
void showCanScreen(bool canOk, unsigned long canInterval, int consecFailures, bool force = false) {
  static bool lastCanOk = false;
  static unsigned long lastCanInterval = 0;
  static int lastConsecFailures = -1;

  if (!force) {
    if (canOk == lastCanOk && canInterval == lastCanInterval && consecFailures == lastConsecFailures) {
      return;
    }
  }

  lastCanOk = canOk;
  lastCanInterval = canInterval;
  lastConsecFailures = consecFailures;
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  if (canOk && consecFailures == 0) {
    display.printf("CAN: OK");
  } else if (!canOk) {
    display.printf("CAN init FAILED!");
  } else {
    display.printf("CAN: FAILED");
  }

  display.setCursor(0, 10);
  display.printf("Fallos de envio: %d", consecFailures);

  display.setCursor(0, 20);
  if (canInterval == 0) {
    display.printf("Ult. msj.: -");
  } else {
    if (canInterval > 999999UL) canInterval = 999999UL;
    display.printf("Ult. msj.: %lums", canInterval);
  }

  display.setCursor(0, 30);
  display.printf("ID envio: 0x603");
  
  display.display();
}

void handleButton() {
  bool st = digitalRead(buttonPin);
  unsigned long now = millis();

  if (st != lastButtonState) {
    // debounce: ignorar cambios si ocurrieron hace menos de debounceMs
    if (now - lastButtonChange < debounceMs) {
      lastButtonChange = now;
      lastButtonState = st;
      return;
    }
    lastButtonChange = now;
    lastButtonState = st;

    if (st == LOW) {
      buttonPressedSince = now;
    } else {
      unsigned long held = now - buttonPressedSince;
      if (held >= longPressMs) {
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.println("Reiniciando...");
        display.display();
        delay(200);
        ESP.restart();
      } else {
        screenIndex = (screenIndex + 1) % 2;
        displayNeedsUpdate = true;
        forceDisplayRedraw = true;
      }
    }
  }
}

void onDataRecv(const uint8_t* mac, const uint8_t* buf, int len) {
  if (len == sizeof(consensus_msg_t)) {
    consensus_msg_t msg;
    memcpy(&msg, buf, len);

    secondLastMessageTime = lastMessageTime;
    lastMessageTime = millis();
    displayNeedsUpdate = true;

    char macs[18];
    sprintf(macs, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.printf("recv from %s  power: %.2f  pri:%u\n", macs, msg.power, msg.priority);

    bool peerFound = false;
    for (auto &p : peers) {
      if (memcmp(p.mac, mac, 6) == 0) {
        p.power = msg.power;
        p.priority = msg.priority;
        p.lastSeen = millis();
        peerFound = true;
        break;
      }
    }
    if (!peerFound) {
      PeerData np;
      memcpy(np.mac, mac, 6);
      np.power = msg.power;
      np.priority = msg.priority;
      np.lastSeen = millis();
      peers.push_back(np);
    }
  }
}

void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = (int)peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > 10 * WINDOW_MS) {
      peers.erase(peers.begin() + i);
      displayNeedsUpdate = true;
    }
  }
}

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

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  lastButtonState = digitalRead(buttonPin);
  lastButtonChange = millis();
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 init failed");
    while (true);
  }
  display.clearDisplay();
  display.display();
  SPI.begin();
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    Serial.println("CAN init failed, continuing but CAN disabled");
    canInitOk = false;
  } else {
    mcp2515.setNormalMode();
    canInitOk = true;
  }
  setupEspNow();
  lastCanAttemptMillis = millis();
  lastAvailabilitySend = 0;
  updateDisplay();
}

void updateDisplay() {
    bool force = forceDisplayRedraw;
    if (screenIndex == 0) {
      float totalConsumption = 0;
      int nodeCount = peers.size();
      for (auto &p : peers) {
        totalConsumption += p.power;
      }
      unsigned long messageInterval = 0;
      if (secondLastMessageTime > 0) {
        messageInterval = lastMessageTime - secondLastMessageTime;
      }
      
      unsigned long minAge = ULONG_MAX;
      int idxMinAge = -1;
      for (int i = 0; i < nodeCount; ++i) {
          unsigned long age = millis() - peers[i].lastSeen;
          if (age < minAge) {
              minAge = age;
              idxMinAge = i;
          }
      }
      char lastNodeMacStr[18] = "-";
      if (idxMinAge >= 0) {
          macToStr(peers[idxMinAge].mac, lastNodeMacStr);
      }
      showMainScreen(200.0f, totalConsumption, nodeCount, messageInterval, (idxMinAge >= 0) ? lastNodeMacStr : nullptr, force);
    } else {
      unsigned long canInterval = 0;
      if (secondLastCanSentTime > 0) {
        canInterval = lastCanSentTime - secondLastCanSentTime;
      }
      showCanScreen(canInitOk, canInterval, canConsecFailures, force);
    }
    displayNeedsUpdate = false;
    forceDisplayRedraw = false;
}

void loop() {
  handleButton();
  unsigned long now = millis();

  // 1) send availability at availabilityInterval
  if (now - lastAvailabilitySend >= availabilityInterval) {
    float availablePower = 200;
    availability_msg_t a = { availablePower };
    esp_now_send(broadcastAddress, (uint8_t*)&a, sizeof(a));
    lastAvailabilitySend = now;
  }

  // 2) purge peers and compute totals
  purgeStalePeers();

  // 3) update display only when a change has been marked
  if (displayNeedsUpdate) {
    updateDisplay();
  }

  // 4) CAN send every canInterval (no-blocking, reintento y recovery)
  if (now - lastCanAttemptMillis >= canInterval) {
    lastCanAttemptMillis = now;
    float totalConsumption = 0;
    for (auto &p : peers) {
      totalConsumption += p.power;
    }

    uint32_t totalConsumption_as_int = static_cast<uint32_t>(totalConsumption * 100);
    canMsg.can_id = 0x603;
    canMsg.can_dlc = 4;
    canMsg.data[0] = (totalConsumption_as_int >> 24) & 0xFF;
    canMsg.data[1] = (totalConsumption_as_int >> 16) & 0xFF;
    canMsg.data[2] = (totalConsumption_as_int >> 8) & 0xFF;
    canMsg.data[3] = totalConsumption_as_int & 0xFF;

    bool messageSent = false;
    int retries = 0;

    if (canInitOk) {
      while (!messageSent && retries < MAX_CAN_RETRIES) {
        if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
          messageSent = true;
          secondLastCanSentTime = lastCanSentTime;
          lastCanSentTime = millis();
          canConsecFailures = 0;
          Serial.printf("CAN send OK  total(centis): %lu  retries:%d\n", (unsigned long)totalConsumption_as_int, retries);
          displayNeedsUpdate = true; // Forzar actualización de pantalla CAN (éxito)
        } else {
          retries++;
          Serial.printf("CAN send fail, retry %d\n", retries);
          struct can_frame tmp;
          if (mcp2515.readMessage(&tmp) == MCP2515::ERROR_OK) {
            Serial.println("Flushed one incoming CAN frame after failed send");
          }
          delay(5);
        }
      }

      if (!messageSent) {
        canConsecFailures++;
        Serial.printf("CAN send: final failure after retries, consecFails=%d\n", canConsecFailures);
        // <<-- CORRECCIÓN: marcar para actualizar la pantalla en cada incremento de fallos
        displayNeedsUpdate = true;
      }

      if (canConsecFailures >= CAN_RESET_THRESHOLD) {
        Serial.println("MCP2515: too many consecutive failures -> resetting MCP2515 and reconfiguring...");
        mcp2515.reset();
        delay(5);
        if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
          Serial.println("MCP2515 reinit OK (8MHz).");
        } else {
          Serial.println("MCP2515 reinit failed with 8MHz, trying 16MHz...");
          if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
            mcp2515.setNormalMode();
            canInitOk = true;
            canConsecFailures = 0;
            Serial.println("MCP2515 reinit OK (16MHz).");
          } else {
            canInitOk = false;
            Serial.println("MCP2515 reinit FAILED (both 8MHz and 16MHz). CAN disabled.");
          }
        }
        displayNeedsUpdate = true; // Forzar actualización si hay un fallo y reintento
      }
    } else {
      static unsigned long lastManualReinitAttempt = 0;
      if (now - lastManualReinitAttempt > 5000) {
        lastManualReinitAttempt = now;
        Serial.println("Attempting MCP2515 init (periodic retry)...");
        mcp2515.reset();
        delay(5);
        if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
          Serial.println("MCP2515 init OK (8MHz).");
        } else if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
          Serial.println("MCP2515 init OK (16MHz).");
        } else {
          Serial.println("MCP2515 still not responding on periodic retry.");
        }
        displayNeedsUpdate = true; // Forzar actualización si se intenta una re-inicialización
      }
    }
  }

  //delay(10);
}
