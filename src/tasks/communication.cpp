#include "communication.h"

#include "../core/Config.h"
#include "../core/Globals.h"

// ------------------------------------------------------------
MCP2515 mcp2515(CAN_CS_PIN);
struct can_frame canMsg;
volatile bool canRxFlag = false;

void IRAM_ATTR canIntISR() {
  canRxFlag = true;
}

// ------------------------------------------------------------
void macToStr(const uint8_t mac[6], char out[18]) {
  sprintf(out, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

// ------------------------------------------------------------
HardwareSerial SerialGW(0); // Usar UART0

static std::vector<PeerData> peers;
static portMUX_TYPE peersMux = portMUX_INITIALIZER_UNLOCKED;

struct ZombiePeer {
    uint8_t mac[6];
    uint8_t priority;
};
static std::vector<ZombiePeer> zombies;

static unsigned long lastPanelRecvTime = 0;
static unsigned long secondLastPanelRecvTime = 0;

static float lastPanelV = 0.0f;
static float lastPanelI = 0.0f;
static float lastPanelP = 0.0f;

static unsigned long lastUartSendMillis = 0;
static unsigned long lastAvailabilitySend = 0;
static unsigned long lastCanAttemptMillis = 0;

static unsigned long lastCanResetAttempt = 0;
static unsigned long lastCanSuccessMillis = 0;

void onDataRecv(const uint8_t* mac, const uint8_t* buf, int len) {
  if (len == sizeof(consensus_msg_t)) {
    consensus_msg_t msg;
    memcpy(&msg, buf, len);
    secondLastMessageTime = lastMessageTime;
    lastMessageTime = millis();
    
    displayNeedsUpdate = true;

    char macStr[18];
    
    macToStr(mac, macStr);
    
    SerialGW.printf("PEER:%s,%.4f,%d\n",
                    macStr,
                    msg.power,
                    msg.priority);

    comm_updatePeer(
            mac,
            msg.power,
            msg.priority);
  }
}

bool comm_InitEspNow(){
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    return false;
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
  return true;
}

bool comm_InitCan(){
  SPI.begin();
  mcp2515.reset();
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) != MCP2515::ERROR_OK) {
    canInitOk = false;
    lastCanResetAttempt = millis();
  } else {
    mcp2515.setNormalMode();
    canInitOk = true;
  }

  if (canInitOk) {
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIntISR, FALLING);
  }

  lastCanAttemptMillis = millis();
  lastCanSuccessMillis = millis();
  lastAvailabilitySend = 0;
  return canInitOk;
}

bool comm_Init(){
    SerialGW.begin(115200,
                   SERIAL_8N1,
                   GW_RX_PIN,
                   GW_TX_PIN);
    comm_InitCan();
    comm_InitEspNow();
    return true;
}    

// ------------------------------------------------------------
void comm_updatePeer( const uint8_t* mac, float power, uint8_t priority){
    portENTER_CRITICAL(&peersMux);
    bool found = false;
    for (auto& p : peers){
        if (memcmp(p.mac, mac, 6) == 0){
            p.power = power;
            p.priority = priority;
            p.secondLastSeen = p.lastSeen;
            p.lastSeen = millis();
            found = true;
            break;
        }
    }
    if (!found){
        PeerData np;
        memcpy(np.mac, mac, 6);
        np.power = power;
        np.priority = priority;
        np.lastSeen = millis();
        np.secondLastSeen = 0;
        peers.push_back(np);
    }
    portEXIT_CRITICAL(&peersMux);

    // Si el nodo vuelve a aparecer, sacarlo de la lista zombie
    for (int i = (int)zombies.size() - 1; i >= 0; --i){
        if (memcmp(zombies[i].mac, mac, 6) == 0){
            zombies.erase(zombies.begin() + i);
            break;
        }
    }

    displayNeedsUpdate = true;
}

void comm_PurgeStalePeers(){
    unsigned long now = millis();
    uint8_t removedMacs[8][6];
    int removedCount = 0;

    uint8_t removedPriorities[8];

    portENTER_CRITICAL(&peersMux);
    for (int i = (int)peers.size() - 1; i >= 0; --i){
        if (now - peers[i].lastSeen > 6 * WINDOW_MS){
            if (removedCount < 8){
                memcpy(removedMacs[removedCount], peers[i].mac, 6);
                removedPriorities[removedCount] = peers[i].priority;
                removedCount++;
            }
            peers.erase(peers.begin() + i);
            displayNeedsUpdate = true;
        }
    }
    portEXIT_CRITICAL(&peersMux);

    for (int i = 0; i < removedCount; i++){
        ZombiePeer z;
        memcpy(z.mac, removedMacs[i], 6);
        z.priority = removedPriorities[i];
        zombies.push_back(z);
    }
}

float comm_GetTotalConsumption(){
    portENTER_CRITICAL(&peersMux);
    float total = 0;
    for (auto& p : peers) total += p.power;
    portEXIT_CRITICAL(&peersMux);
    return total;
}

int comm_GetPeerCount(){
    portENTER_CRITICAL(&peersMux);
    int count = peers.size();
    portEXIT_CRITICAL(&peersMux);
    return count;
}

// retorna copia para que el llamador itere sin riesgo de race condition
std::vector<PeerData> comm_GetPeers(){
    portENTER_CRITICAL(&peersMux);
    auto copy = peers;
    portEXIT_CRITICAL(&peersMux);
    return copy;
}

PeerData comm_FindNewestPeer(){
    portENTER_CRITICAL(&peersMux);
    PeerData result = {};
    unsigned long minAge = ULONG_MAX;
    for (auto& p : peers){
        unsigned long age = millis() - p.lastSeen;
        if (age < minAge){ minAge = age; result = p; }
    }
    portEXIT_CRITICAL(&peersMux);
    return result;
}

// ------------------------------------------------------------

void comm_processUart(){
    unsigned long now = millis();

    if (now - lastUartSendMillis >= uartSendInterval) {
        lastUartSendMillis = now;

        // 1. Enviar VRMS y Corriente Disponible
        float current_available_uart = 0.0f;
        if (Vrms >= V_MIN && availablePower > 0.0f && panelDataValid)
            current_available_uart = availablePower / Vrms;
        SerialGW.printf("VRMS:%.2f,%.3f\n", Vrms, current_available_uart);

        // 2. Enviar Potencia del CAN
        float power_to_send_uart = (panelDataValid && availablePower > 0.0f) ? availablePower : 0.0f;
        SerialGW.printf("CAN_PWR:%.1f\n", power_to_send_uart);

        // 3. Anunciar consumo=0 para nodos desaparecidos, conservando su prioridad real
        for (auto& z : zombies){
            char macStr[18];
            macToStr(z.mac, macStr);
            SerialGW.printf("PEER:%s,0.0000,%d\n", macStr, z.priority);
        }
    }
}

void comm_processEspNow(){
    unsigned long now = millis();
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
}

// ------------------------------------------------------------

void comm_processCanRx(){
  if (!canInitOk) return;
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

        // reconstrucción float con la misma escala que usa el AG (0..4095 -> 0..V_MAX / 0..I_MAX)
        float measV = ((float)v_u16) / 4095.0f * V_MAX;
        float measI = ((float)i_u16) / 4095.0f * I_MAX;

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
}

// Reinicia el MCP2515 y re-engancha el ISR.
// Si falla, el sistema reintentará cada CAN_RETRY_INTERVAL_MS indefinidamente.
static bool comm_resetCan() {
  detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN));
  canRxFlag = false;

  mcp2515.reset();
  delay(50);

  bool ok = false;
  if (mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ) == MCP2515::ERROR_OK) {
    ok = true;
  } else if (mcp2515.setBitrate(CAN_500KBPS, MCP_16MHZ) == MCP2515::ERROR_OK) {
    ok = true;
  }

  if (ok) {
    mcp2515.setNormalMode();
    canInitOk         = true;
    canConsecFailures = 0;
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIntISR, FALLING);
  } else {
    canInitOk = false;
  }

  lastCanResetAttempt = millis();
  displayNeedsUpdate  = true;
  return ok;
}

void comm_processCanTx() {
  unsigned long now = millis();

  // --- Módulo fuera de servicio: reintentar cada CAN_RETRY_INTERVAL_MS indefinidamente ---
  if (!canInitOk) {
    if (now - lastCanResetAttempt >= CAN_RETRY_INTERVAL_MS) {
      comm_resetCan();
    }
    return;
  }

  // --- Demasiados fallos consecutivos O sin éxito por demasiado tiempo ---
  bool forceReset = canConsecFailures >= CAN_RESET_THRESHOLD ||
                    (now - lastCanSuccessMillis > CAN_FORCE_RESET_MS);
  if (forceReset) {
    detachInterrupt(digitalPinToInterrupt(CAN_INT_PIN));
    canRxFlag           = false;
    canInitOk           = false;
    lastCanResetAttempt = now;
    displayNeedsUpdate  = true;
    return;
  }

  // --- Envío periódico ---
  if (now - lastCanAttemptMillis < canInterval) return;
  lastCanAttemptMillis = now;

  float totalConsumption = comm_GetTotalConsumption();
  uint16_t totalConsumption_as_int = static_cast<uint16_t>(totalConsumption * 100);
  canMsg.can_id  = 0x620;
  canMsg.can_dlc = 2;
  canMsg.data[0] = totalConsumption_as_int & 0xFF;
  canMsg.data[1] = (totalConsumption_as_int >> 8) & 0xFF;

  bool messageSent = false;
  for (int retries = 0; !messageSent && retries < MAX_CAN_RETRIES; retries++) {
    if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
      messageSent           = true;
      secondLastCanSentTime = lastCanSentTime;
      lastCanSentTime       = now;
      lastCanSuccessMillis  = now;
      canConsecFailures     = 0;
      displayNeedsUpdate    = true;
    } else {
      struct can_frame tmp;
      mcp2515.readMessage(&tmp);
    }
  }

  if (!messageSent) {
    canConsecFailures++;
    displayNeedsUpdate = true;
  }
}

// ------------------------------------------------------------

void comm_Process()
{
    comm_processCanRx();

    if(panelDataValid &&
       millis() - lastPanelRecvTime >
       DATA_EXPIRY_MS)
    {
        panelDataValid = false;
        displayNeedsUpdate = true;
    }

    comm_processUart();
    comm_processEspNow();
    comm_processCanTx();
    comm_PurgeStalePeers();
}

