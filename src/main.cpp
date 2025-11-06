#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <SPI.h>
#include <MCP2515.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <cstring>
#include <driver/adc.h>
#include "esp_adc_cal.h"

static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ------------------------------ estructura mensaje de disponibilidad -------------------------------------------
struct availability_msg_t {
  float availableCurrent; 
};

// --------------------------------- estructura mensaje de consenso ----------------------------------------------
struct consensus_msg_t {
  float power;
  uint8_t priority;
};

// ---------------------------- estructura para lista de nodos de consumo ----------------------------------------
struct PeerData {
  uint8_t mac[6];
  float power;
  uint8_t priority;
  unsigned long lastSeen;
  unsigned long secondLastSeen; // Nuevo: tiempo del penúltimo mensaje
};

static std::vector<PeerData> peers;
const unsigned long WINDOW_MS = 600; // ventana para considerar nodos activos
static unsigned long lastMessageTime = 0;
static unsigned long secondLastMessageTime = 0;

// ------------------------------------------- pantalla OLED -----------------------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----------------------------------- Variables para comunicacion CAN -------------------------------------------
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4
#define MAX_CAN_RETRIES 3
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canMsg;
static int canConsecFailures = 0;
const int CAN_RESET_THRESHOLD = 5;
static unsigned long lastCanSentTime = 0;
static unsigned long secondLastCanSentTime = 0;
bool canInitOk = false;
unsigned long lastCanAttemptMillis = 0;
unsigned long lastAvailabilitySend = 0;
const unsigned long availabilityInterval = WINDOW_MS; 
const unsigned long canInterval = 1000;

// -------------------- parámetros de escalado para los mensajes recibidos desde la placa TI ----------------------
#define V_MAX 100.0f   
#define I_MAX 40.0f    

// ------------------------------ botones ------------------------------------------------
const int btnPrevPin = 14; 
const int btnNextPin = 27; 
const int btnHomePin = 26; 

volatile bool btnPrevPressed = false;
volatile bool btnNextPressed = false;
volatile bool btnHomePressed = false;
volatile unsigned long btnHomePressStart = 0;
volatile unsigned long lastButtonInterrupt[3] = {0, 0, 0};

const unsigned long debounceMs = 25;
const unsigned long longPressMs = 3000;

// ------------------------------ variables para manejo de las pantallas -----------------------------------------
int screenIndex = 0;
bool displayNeedsUpdate = true;
volatile bool forceDisplayRedraw = false;

// scroll para lista de nodos
const int nodesPerPage = 5;
int nodesStartIndex = 0;

// ------------------------------ variables para calculo de potencia ----------------------------------------------
// OBSERVACIÓN: availablePower ahora -1.0f indica "sin dato CAN válido todavía"
const float V_MIN = 30.0f;
double Vrms = 0;
float availablePower = -1.0f; // valor por defecto; -1 = sin dato CAN
bool panelDataValid = false;   // indica si hay dato CAN válido y no caducado
const unsigned long DATA_EXPIRY_MS = 10000; // 10s: si no llega msg en 10 s, considerar sin dato

unsigned long time_perfil = millis();
int i = 0;
int count = 0;

// ADC sampling parameters
const int sensorPin = 33;
const adc1_channel_t adcChannel = ADC1_CHANNEL_5;
const adc_atten_t adcAtten = ADC_ATTEN_DB_12;
const adc_bits_width_t adcWidth = ADC_WIDTH_BIT_12;

// cantidad de muestras a tomar para evitar efecto de ventaneo
const int Fs = 50000; 
const int lineFreq = 50;
const int samplesPerPeriod = Fs / lineFreq;
const int periodsToCapture = 10;
const int bufferSize = samplesPerPeriod * periodsToCapture;

static volatile uint16_t adcBuffer[bufferSize];
static volatile int bufferIndex = 0;
static volatile bool bufferFull = false;
static float voltageOffset = 0.0f;

hw_timer_t* samplingTimer = nullptr;
esp_adc_cal_characteristics_t adc_chars;
             
// ---------------------------------------- declaracion de funciones ----------------------------------------------
void updateDisplay();
void macToStr(const uint8_t mac[6], char out[18]);
void showMainScreen(float availablePower, float V_rms, float availableCurrent, float totalConsumption, int nodeCount, unsigned long messageInterval, const char* lastNodeMac, bool force, bool panelValid);
void showCanScreen(bool canOk, unsigned long canInterval, int consecFailures, bool force);
void showNodesScreenPaged(int startIndex);
void purgeStalePeers();
void setupEspNow();
void displayMessage(const char* msg, bool clear = true);

// ------------------------------------------ ISR y funciones del ADC ----------------------------------------------
void IRAM_ATTR onTimerCallback() {
  if (!bufferFull && bufferIndex < bufferSize) {
    int raw = adc1_get_raw(adcChannel);
    adcBuffer[bufferIndex] = (uint16_t)raw;
    bufferIndex = bufferIndex + 1;
    if (bufferIndex >= bufferSize) {
      bufferFull = true;
    }
  }
}

void startSampling() {
  bufferIndex = 0;
  bufferFull = false;
  timerAlarmDisable(samplingTimer);
  timerAlarmWrite(samplingTimer, 1000000UL / Fs, true);
  timerAlarmEnable(samplingTimer);
}

void stopSampling() {
  timerAlarmDisable(samplingTimer);
}

void macToStr(const uint8_t mac[6], char out[18]) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// -------------------------------------------- funciones de pantalla ---------------------------------------------
void displayMessage(const char* msg, bool clear) {
  if (clear) display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.println(msg);
  display.display();
}

// ------------------------------------- pantalla 0: disponibilidad y consumo --------------------------------------
void showMainScreen(float availablePower, float V_rms, float availableCurrent, float totalConsumption, int nodeCount, unsigned long messageInterval, const char* lastNodeMac, bool force, bool panelValid) {
  static float lastAvailablePower = -9999.0f;
  static float lastV = -1.0f;
  static float lastI = -1.0f;
  static float lastTotal = -1.0f;
  static int lastNodeCount = -1;
  static unsigned long lastMsgInterval = 0;
  static char lastMacPrinted[18] = "-";
  static bool lastPanelValid = false;

  if (!force) {
    if (fabs(availablePower - lastAvailablePower) < 0.1f &&
        fabs(V_rms - lastV) < 0.01f &&
        fabs(availableCurrent - lastI) < 0.01f &&
        fabs(totalConsumption - lastTotal) < 0.1f &&
        nodeCount == lastNodeCount &&
        messageInterval == lastMsgInterval &&
        ((lastNodeMac==nullptr && strcmp(lastMacPrinted,"-")==0) || (lastNodeMac && strcmp(lastNodeMac,lastMacPrinted)==0)) &&
        panelValid == lastPanelValid) {
      return;
    }
  }

  lastAvailablePower = availablePower;
  lastV = V_rms;
  lastI = availableCurrent;
  lastTotal = totalConsumption;
  lastNodeCount = nodeCount;
  lastMsgInterval = messageInterval;
  lastPanelValid = panelValid;

  if (lastNodeMac) {
    strncpy(lastMacPrinted, lastNodeMac, sizeof(lastMacPrinted)-1);
    lastMacPrinted[sizeof(lastMacPrinted)-1] = '\0';
  } else {
    strcpy(lastMacPrinted,"-");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (!panelValid || availablePower < 0.0f) {
    display.printf("Potencia: N/A");
  } else {
    display.printf("Potencia: %.1f W", availablePower);
  }

  display.setCursor(0, 10);
  display.printf("Vrms: %.2f V", V_rms);

  display.setCursor(0, 20);
  if (!panelValid || availablePower < 0.0f) {
    display.printf("Imax: -- A");
  } else {
    display.printf("Imax: %.3f A", availableCurrent);
  }

  display.setCursor(0, 40);
  display.printf("Consumo: %.4f A", totalConsumption);

  display.display();
}

// --------------------------------------- pantalla 1: comunicacion CAN ---------------------------------------
void showCanScreen(bool canOk_local, unsigned long canInterval_local, int consecFailures, bool force) {
  static bool lastCanOk = false;
  static unsigned long lastCanInterval = 0;
  static int lastConsec = -1;

  if (!force) {
    if (canOk_local == lastCanOk && canInterval_local == lastCanInterval && consecFailures == lastConsec) {
      return;
    }
  }

  lastCanOk = canOk_local;
  lastCanInterval = canInterval_local;
  lastConsec = consecFailures;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  if (canOk_local && consecFailures == 0) {
    display.printf("CAN: OK");
  } else if (!canOk_local) {
    display.printf("CAN init FAILED!");
  } else {
    display.printf("CAN: FAILED");
  }

  display.setCursor(0, 10);
  display.printf("Fallos: %d", consecFailures);

  display.setCursor(0, 20);
  if (canInterval_local == 0) display.printf("Ult. msj: -");
  else display.printf("Ult. msj: %lums", canInterval_local > 999999UL ? 999999UL : canInterval_local);

  display.setCursor(0, 30);
  display.printf("ID envio: 0x603");

  display.display();
}

// --------------------------------------- pantalla 2: estado de los nodos de Consumo ---------------------------------------
void showNodesScreenPaged(int startIndex) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("Nodos: %d", peers.size());

  unsigned long now = millis();
  int y = 10;
  int total = peers.size();
  int endIdx = min(total, startIndex + nodesPerPage);
  for (int i = startIndex; i < endIdx; ++i) {
    char macs[18];
    macToStr(peers[i].mac, macs);
    display.setCursor(0, y);
    display.printf("%.8s", macs); 
    display.setCursor(64, y);
    display.printf("%.1fA", peers[i].power);
    y += 8;
    display.setCursor(0, y);
    if (peers[i].secondLastSeen == 0) {
      unsigned long age = now - peers[i].lastSeen;
      display.printf("age:%lums pri:%u", age, peers[i].priority);
    } else {
      unsigned long interval = peers[i].lastSeen - peers[i].secondLastSeen;
      display.printf("age:%lums pri:%u", interval, peers[i].priority);
    }
    y += 10;
  }

  display.display();
}

// --------------------------------------- actualizacion de la pantalla ---------------------------------------
void updateDisplay() {
    bool force = forceDisplayRedraw;
    if (screenIndex == 0) {
      float totalConsumption = 0;
      int nodeCount = peers.size();
      for (auto &p : peers) totalConsumption += p.power;
      unsigned long messageInterval = 0;
      if (secondLastMessageTime > 0) messageInterval = lastMessageTime - secondLastMessageTime;

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
      if (idxMinAge >= 0) macToStr(peers[idxMinAge].mac, lastNodeMacStr);

      float availableCurrent = 0.0f;
      if (Vrms >= V_MIN && availablePower > 0.0f && panelDataValid) {
        availableCurrent = availablePower / Vrms;
      } else {
        availableCurrent = 0.0f;
      }
      showMainScreen(availablePower, Vrms, availableCurrent, totalConsumption, nodeCount, messageInterval, (idxMinAge >= 0) ? lastNodeMacStr : nullptr, force, panelDataValid);
    } else if (screenIndex == 1) {
      showNodesScreenPaged(nodesStartIndex);
    } else {
      unsigned long canIntervalLocal = 0;
      if (secondLastCanSentTime > 0) canIntervalLocal = lastCanSentTime - secondLastCanSentTime;
      showCanScreen(canInitOk, canIntervalLocal, canConsecFailures, force);
    }
    displayNeedsUpdate = false;
    forceDisplayRedraw = false;
}

// ---------------------------------------------- ISR de botones ----------------------------------------------------

void IRAM_ATTR isrPrev() {
  lastButtonInterrupt[0] = millis();
}

void IRAM_ATTR isrNext() {
  lastButtonInterrupt[1] = millis();
}

void IRAM_ATTR isrHome() {
  btnHomePressStart = millis();
  btnHomePressed = true;
}

void checkButtons() {
    unsigned long now = millis();
    // boton anterior
    if (digitalRead(btnPrevPin) == HIGH && btnPrevPressed) {
        if (now - lastButtonInterrupt[0] > debounceMs) {
            btnPrevPressed = false;
        }
    } else if (digitalRead(btnPrevPin) == LOW && now - lastButtonInterrupt[0] > debounceMs) {
        if (!btnPrevPressed) {
            btnPrevPressed = true;
            lastButtonInterrupt[0] = now;
            if (screenIndex > 0) {
                screenIndex--;
            } else {
                screenIndex = 2;
            }
            nodesStartIndex = 0;
            displayNeedsUpdate = true;
            forceDisplayRedraw = true;
        }
    }
    // boton siguiente
    if (digitalRead(btnNextPin) == HIGH && btnNextPressed) {
        if (now - lastButtonInterrupt[1] > debounceMs) {
            btnNextPressed = false;
        }
    } else if (digitalRead(btnNextPin) == LOW && now - lastButtonInterrupt[1] > debounceMs) {
        if (!btnNextPressed) {
            btnNextPressed = true;
            lastButtonInterrupt[1] = now;
            if (screenIndex < 2) {
                screenIndex++;
            } else {
                screenIndex = 0; 
            }
            nodesStartIndex = 0;
            displayNeedsUpdate = true;
            forceDisplayRedraw = true;
        }
    }
    
    //reiniciar con pulsacion larga 
    if (digitalRead(btnHomePin) == LOW && (now - btnHomePressStart) >= longPressMs && btnHomePressed) {
      btnHomePressed = false;
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.println("Reiniciando...");
      display.display();
      delay(200);
      ESP.restart();
    }
    
    if (digitalRead(btnHomePin) == HIGH) {
        btnHomePressed = false;
    }
}

// ---------------------------------------- funciones ESP-NOW -----------------------------------------------------
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
    
    bool peerFound = false;
    for (auto &p : peers) {
      if (memcmp(p.mac, mac, 6) == 0) {
        p.power = msg.power;
        p.priority = msg.priority;
        p.secondLastSeen = p.lastSeen; // Guardar el valor anterior
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
      np.secondLastSeen = 0; // Se inicializa a 0
      peers.push_back(np);
      displayNeedsUpdate = true;
    }
  }
}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
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

// ---------------------------------------- limpieza de nodos viejos -------------------------------------------
void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = (int)peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > 10 * WINDOW_MS) {
      peers.erase(peers.begin() + i);
      displayNeedsUpdate = true;
      int maxStart = max(0, (int)peers.size() - nodesPerPage);
      if (nodesStartIndex > maxStart) nodesStartIndex = maxStart;
    }
  }
}

// ------------------------------------ ISR mesajes CAN ------------------------------------
volatile bool canRxFlag = false;          // bandera puesta cuando MCP2515 activa INT
unsigned long lastPanelRecvTime = 0;
unsigned long secondLastPanelRecvTime = 0;
float lastPanelV = 0.0f;
float lastPanelI = 0.0f;
float lastPanelP = 0.0f;

void IRAM_ATTR canIntISR() {
  canRxFlag = true;
}

// ---------------------------------------------- setup y loop ------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // ADC setup
  analogSetPinAttenuation(sensorPin, ADC_11db);
  adc1_config_width(adcWidth);
  adc1_config_channel_atten(adcChannel, adcAtten);
  esp_adc_cal_characterize(ADC_UNIT_1, adcAtten, adcWidth, 1100, &adc_chars);

  // configure timer for ADC sampling (prescaler 80 -> 1 MHz ticks)
  samplingTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(samplingTimer, &onTimerCallback, true);

  // Button setup (Interrupciones)
  pinMode(btnPrevPin, INPUT_PULLUP);
  pinMode(btnNextPin, INPUT_PULLUP);
  pinMode(btnHomePin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(btnPrevPin), isrPrev, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnNextPin), isrNext, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnHomePin), isrHome, FALLING);

  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (true);
  }
  display.clearDisplay();
  display.display();

  SPI.begin();
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    canInitOk = false;
  } else {
    mcp2515.setNormalMode();
    canInitOk = true;
  }

  // Interrupcion de mensaje CAN, flanco bajo en el pin 4
  if (canInitOk) {
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIntISR, FALLING);
  }

  setupEspNow();

  lastCanAttemptMillis = millis();
  lastAvailabilitySend = 0;

  // calibración de offset
  startSampling();
  while (!bufferFull) {
    yield();
  }
  stopSampling();

  uint64_t sum_mV = 0;
  for (int i = 0; i < bufferIndex; ++i) {
    uint32_t mV = esp_adc_cal_raw_to_voltage(adcBuffer[i], &adc_chars);
    sum_mV += mV;
  }
  voltageOffset = float(sum_mV) / float(bufferIndex) / 1000.0f;

  bufferFull = false;
  bufferIndex = 0;
  delay(200);
  startSampling();

  updateDisplay();
   
}

void loop() {

  unsigned long now = millis();

  // Manejo de botones con flags y lógica fuera de ISR
  checkButtons();

  // ---------- Computo Vrms cuando el buffer está lleno ----------
  if (bufferFull) {
    stopSampling();
    double sumsq = 0.0;
    for (int i = 0; i < bufferSize; ++i) {
      uint32_t mV = esp_adc_cal_raw_to_voltage(adcBuffer[i], &adc_chars);
      double V = double(mV) / 1000.0;
      double Vcorr = V - voltageOffset;
      sumsq += Vcorr * Vcorr;
    }
    Vrms = sqrt(sumsq / double(bufferSize));
    Vrms = Vrms*998; // mantener la escala original (revisar si es necesario)

    // reiniciar muestreo
    bufferIndex = 0;
    bufferFull = false;
    startSampling();
    displayNeedsUpdate = true;
  }


  // ---------- PROCESAR MENSAJES CAN RECIBIDOS ----------
  if (canRxFlag) {
    // limpiar flag lo antes posible
    canRxFlag = false;

    struct can_frame tmpFrame;
    // leer todos los mensajes que estén pendientes
    while (mcp2515.readMessage(&tmpFrame) == MCP2515::ERROR_OK) {
      uint16_t recvId = (uint16_t)(tmpFrame.can_id & 0x7FF);
      if (recvId == 0x610 && tmpFrame.can_dlc >= 4) {
       // parámetros de validación
        const float MIN_VALID_V = 0.01f;   // V umbral para considerar "no cero"
        const float MIN_VALID_I = 0.01f;   // A umbral para considerar "no cero"

        //desempaquetar datos
        uint16_t v_u16 = (uint16_t)( (uint16_t)tmpFrame.data[0] | ((uint16_t)tmpFrame.data[1] << 8) );
        uint16_t i_u16 = (uint16_t)( (uint16_t)tmpFrame.data[2] | ((uint16_t)tmpFrame.data[3] << 8) );

        // reconstrucción float con la misma escala que usa el AG (0..65535 -> 0..V_MAX / 0..I_MAX)
        float measV = ((float)v_u16) / 65535.0f * V_MAX;
        float measI = ((float)i_u16) / 65535.0f * I_MAX;

        // clausulas por si llegan números fuera de rango 
        if (measV < 0.0f) measV = 0.0f;
        if (measI < 0.0f) measI = 0.0f;
        if (measV > V_MAX) measV = V_MAX;
        if (measI > I_MAX) measI = I_MAX;

        // Si ambos valores son muy pequeños, tratarlos como cero
        if (measV < MIN_VALID_V) measV = 0.0f;
        if (measI < MIN_VALID_I) measI = 0.0f;
        
        //valor de potencia
        float panelP = measV * measI;

        // actualizar timestamps
        secondLastPanelRecvTime = lastPanelRecvTime;
        lastPanelRecvTime = millis();

        lastPanelV = measV;
        lastPanelI = measI;
        lastPanelP = panelP;

        // actualizar last known data y marcar como válido
        availablePower = panelP;
        panelDataValid = true;

        // actualizar display
        displayNeedsUpdate = true;
      }
    }
  }

  // ---------- Envio y recepción de información local ----------
  // invalidar panelDataValid si ha pasado mucho tiempo desde el último mensaje CAN
  if (panelDataValid && (millis() - lastPanelRecvTime > DATA_EXPIRY_MS)) {
    panelDataValid = false;
    // Si prefieres borrar también el valor guardado, descomenta la siguiente línea:
    // availablePower = -1.0f;
    displayNeedsUpdate = true;
  }

  if (now - lastAvailabilitySend >= availabilityInterval) {
    float availableCurrent = 0.0f;
    if (Vrms >= V_MIN && availablePower > 0.0f && panelDataValid) {
      availableCurrent = availablePower / Vrms;
    } else {
      availableCurrent = 0.0f;
    }

    availability_msg_t a = { availableCurrent };
    esp_now_send(broadcastAddress, (uint8_t*)&a, sizeof(a));
    lastAvailabilitySend = now;
  }

  // purgado de nodos viejos
  purgeStalePeers();

  // actualizar display si es necesario
  if (displayNeedsUpdate) updateDisplay();

  // ---------- Envio de mensajes CAN ----------
  if (now - lastCanAttemptMillis >= canInterval) {
    lastCanAttemptMillis = now;
    float totalConsumption = 0;
    for (auto &p : peers) totalConsumption += p.power;

    uint32_t totalConsumption_as_int = static_cast<uint32_t>(totalConsumption * 100);
    canMsg.can_id = 0x620;
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
          displayNeedsUpdate = true;
        } else {
          retries++;
          struct can_frame tmp;
          if (mcp2515.readMessage(&tmp) == MCP2515::ERROR_OK) {
          }
        }
      }

      if (!messageSent) {
        canConsecFailures++;
        displayNeedsUpdate = true;
      }

      if (canConsecFailures >= CAN_RESET_THRESHOLD) {
        mcp2515.reset();
        if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
        } else {
          if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
            mcp2515.setNormalMode();
            canInitOk = true;
            canConsecFailures = 0;
          } else {
            canInitOk = false;
          }
        }
        displayNeedsUpdate = true;
      }
    } else {
      static unsigned long lastManualReinitAttempt = 0;
      if (now - lastManualReinitAttempt > 5000) {
        lastManualReinitAttempt = now;
        mcp2515.reset();
        if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
        } else if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
          mcp2515.setNormalMode();
          canInitOk = true;
          canConsecFailures = 0;
        } else {
        }
        displayNeedsUpdate = true;
      }
    }
  }
}
