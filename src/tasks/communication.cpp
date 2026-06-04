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

static unsigned long lastPanelRecvTime = 0;
static unsigned long secondLastPanelRecvTime = 0;

static float lastPanelV = 0.0f;
static float lastPanelI = 0.0f;
static float lastPanelP = 0.0f;

static unsigned long lastUartSendMillis = 0;
static unsigned long lastAvailabilitySend = 0;
static unsigned long lastCanAttemptMillis = 0;

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

    char macs[18];
    sprintf(macs, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    
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
    while (true) delay(1000);
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
  } else {
    mcp2515.setNormalMode();
    canInitOk = true;
  }

  // Interrupcion de mensaje CAN, flanco bajo en el pin 4
  if (canInitOk) {
    pinMode(CAN_INT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), canIntISR, FALLING);
  }

  lastCanAttemptMillis = millis();
  lastAvailabilitySend = 0;

  return true;

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
    displayNeedsUpdate = true;
}

void comm_PurgeStalePeers(){
    unsigned long now = millis();
    for (int i = (int)peers.size() - 1; i >= 0; --i){
        if (now - peers[i].lastSeen > 10 * WINDOW_MS){
            peers.erase(peers.begin() + i);
            displayNeedsUpdate = true;
        }
    }
}

float comm_GetTotalConsumption(){
    float total = 0;
    for (auto& p : peers){
        total += p.power;
    }
    return total;
}

int comm_GetPeerCount(){
    return peers.size();
}

std::vector<PeerData>& comm_GetPeers(){
    return peers;
}

PeerData* comm_FindNewestPeer(){
    if (peers.empty()){
        return nullptr;
    }
    unsigned long minAge = ULONG_MAX;
    PeerData* newest = nullptr;
    for (auto& p : peers){
        unsigned long age = millis() - p.lastSeen;

        if (age < minAge){
            minAge = age;
            newest = &p;
        }
    }
    return newest;
}

// ------------------------------------------------------------

void comm_processUart(){
    unsigned long now = millis();
    if (now - lastUartSendMillis >= uartSendInterval) {
        lastUartSendMillis = now;

        // 1. Enviar VRMS y Corriente Disponible
        float current_available_uart = 0.0f;
        if (Vrms >= V_MIN && availablePower > 0.0f && panelDataValid) {
        current_available_uart = availablePower / Vrms;
        }
        // Formato: VRMS:voltaje,corriente
        SerialGW.printf("VRMS:%.2f,%.3f\n", Vrms, current_available_uart);

        // 2. Enviar Potencia del CAN
        // Usamos el valor de 'availablePower' global, asegurando enviar 0 si no es válido
        float power_to_send_uart = (panelDataValid && availablePower > 0.0f) ? availablePower : 0.0f;
        // Formato: CAN_PWR:potencia
        SerialGW.printf("CAN_PWR:%.1f\n", power_to_send_uart);
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

void comm_processCanTx(){
    unsigned long now = millis();
  // ---------- Envio de mensajes CAN ----------
  if (now - lastCanAttemptMillis >= canInterval) {
    lastCanAttemptMillis = now;
    float totalConsumption = comm_GetTotalConsumption();

    uint16_t totalConsumption_as_int = static_cast<uint16_t>(totalConsumption * 100); 
    canMsg.can_id = 0x620;
    canMsg.can_dlc = 2;
    // little-endian (LSB primero)
    canMsg.data[0] = totalConsumption_as_int  & 0xFF;   // LSB
    canMsg.data[1] = (totalConsumption_as_int >> 8)  & 0xFF;   // MSB

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

