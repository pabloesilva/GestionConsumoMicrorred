#include <esp_now.h>
#include <WiFi.h>
#include <vector>

#define CONVERSIONS_PER_PIN 3

// esp_adc_cal_characteristics_t adc_chars;

// direccion broadcast para ESP-NOW
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//estructura que llega del nodo informante
typedef struct {
  float availablePower;   // potencia total disponible (W)
} availability_msg_t;

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

// almacenamiento de potencias recibidas
static std::vector<PeerData> peers;

// ventana de recepción en milisegundos
const unsigned long WINDOW_MS = 200;

// Pines y configuración
const int sensorPin = 34;                       // GPIO34 -> ADC1_CHANNEL_6
const int Interruptor0 = 14;
const int Interruptor1 = 26;
const int rele = 33;
const int ledRojo1 = 15, ledRojo2 = 4;
const int ledAmarillo1 = 5, ledAmarillo2 = 19;
const int ledVerde1 = 22, ledVerde2 = 23;

// array con los pines en orden
const int ledPins[6] = {
  ledRojo1,
  ledRojo2,
  ledAmarillo1,
  ledAmarillo2,
  ledVerde1,
  ledVerde2
};

// Constantes de Cálculo
const float voltageRMS = 220.0f;

// Parámetros de muestreo
const float lineFreq = 50;
const int Fs = 20000;
const int samplesPerPeriod = Fs / lineFreq;
const int periodsToCapture = 50;
const int bufferSize = samplesPerPeriod * periodsToCapture;

// Buffers y flags
static volatile uint16_t adcBuffer[bufferSize];
static volatile int bufferIndex = 0;
static volatile bool bufferFull = false;
static float voltageOffset = 0.0f;

//parármetros del ADC continuo
uint8_t adc_pins[] = {sensorPin}; 
uint8_t adc_pins_count = 1;
volatile bool adc_coversion_done = false;
// volatile int adc_coversion_count = 0;
// const int Fs = 20000;

adc_continuous_data_t * result = NULL;

// ISR conversión completa ADC
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
  // adc_coversion_count++;
}

// callback de recepción ESP-NOW
void onDataRecv(const esp_now_recv_info_t * info, const uint8_t* buf, int len) {
  uint8_t mac[6];
  memcpy(mac, info->src_addr, 6);
  //mensaje de disponibilidad
  if (len == sizeof(availability_msg_t)) {
    availability_msg_t msg;
    memcpy(&msg, buf, len);

    // definimos la franja de cada led
    const float maxPower = 2200.0f;
    const float segment = maxPower / 6.0f;  // ~366.67 W por led

    // calculamos cuÃ¡ntos leds encender
    int ledsOn = int(msg.availablePower / segment + 0.0001f);
    if (ledsOn > 6) ledsOn = 6;
    if (ledsOn < 0) ledsOn = 0;

    // actualizar estados
    for (int i = 0; i < 6; ++i) {
      digitalWrite(ledPins[i], (i < ledsOn) ? HIGH : LOW);
    }
    return;
  }
  
  // mensaje de concenso
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
    if (now - peers[i].lastSeen > 10*WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

void setup() {
    Serial.begin(115200);

    // Configurar pines
    pinMode(sensorPin, INPUT);
    pinMode(Interruptor0, INPUT);
    pinMode(Interruptor1, INPUT);
    pinMode(rele, OUTPUT);
    pinMode(ledRojo1, OUTPUT);
    pinMode(ledRojo2, OUTPUT);
    pinMode(ledAmarillo1, OUTPUT);
    pinMode(ledAmarillo2, OUTPUT);
    pinMode(ledVerde1, OUTPUT);
    pinMode(ledVerde2, OUTPUT);

    // Configurar ADC
    analogContinuousSetWidth(12);
    analogContinuousSetAtten(ADC_11db);
    analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, Fs, &adcComplete);
    
    

    // --- CALIBRACIÓN DE OFFSET ---
    // Encender LEDs 
    digitalWrite(ledRojo1, HIGH);
    delay(200);
    digitalWrite(ledRojo2, HIGH);
    delay(200);
    digitalWrite(ledAmarillo1, HIGH);
    delay(200);
    digitalWrite(ledAmarillo2, HIGH);
    delay(200);
    digitalWrite(ledVerde1, HIGH);
    delay(200);
    digitalWrite(ledVerde2, HIGH);

    Serial.println("Iniciando calibración de offset...");
    // Iniciar la conversión continua del ADC
    analogContinuousStart();
    while (!bufferFull) {
      if (adc_coversion_done){
        adc_coversion_done = false;
        if (analogContinuousRead(&result, 0)){
          adcBuffer[bufferIndex++] = result[0].avg_read_mvolts;
          if (bufferIndex >= bufferSize) {
            bufferFull = true;
            bufferIndex = 0;
          }
        }
      }
    }
    analogContinuousStop();

    float sum = 0.0f;
    for (int i = 0; i < bufferSize; i++) {
      sum += adcBuffer[i] / 1000.0f;  // convertir a voltios
    }
    voltageOffset = sum / bufferSize;
    bufferFull = false;
    Serial.printf("Offset calibrado: %.5f V\n", voltageOffset);

    // Apagar LEDs
    delay(1000);
    digitalWrite(ledRojo1, LOW);
    delay(200);
    digitalWrite(ledRojo2, LOW);
    delay(200);
    digitalWrite(ledAmarillo1, LOW);
    delay(200);
    digitalWrite(ledAmarillo2, LOW);
    delay(200);
    digitalWrite(ledVerde1, LOW);
    delay(200);
    digitalWrite(ledVerde2, LOW);

    // configurar wifi en modo station para ESP-NOW
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // inicializar ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("error inicializando esp-now");
      return;
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

    // Iniciar la conversión continua del ADC
    analogContinuousStart();

}

void loop() {
  
  int estado = digitalRead(Interruptor0);
  digitalWrite(rele, (estado == HIGH) ? LOW : HIGH);

  uint8_t myPriority = 0;                     // ejemplo: prioridad alta

  if (adc_coversion_done){
    adc_coversion_done = false;
    if (analogContinuousRead(&result, 0)){
      adcBuffer[bufferIndex++] = result[0].avg_read_mvolts;
      if (bufferIndex >= bufferSize) {
        bufferFull = true;
        bufferIndex = 0;
      }
  }
  }


  if (bufferFull) {
    analogContinuousStop();
    bufferFull = false;
    
    float sumsq = 0.0;
  
    for (int i = 0; i < bufferSize; i++) {
      float V = adcBuffer[i] / 1000.0f;  // convertir a voltios
      V = V - voltageOffset;
      sumsq += V * V;
    }
    float Vrms = sqrt(sumsq / bufferSize);
    // Vrms = Vrms - 0.0135;
    // Vrms = (Vrms < 0.0005) ? 0 : Vrms;
    float currentRMS = Vrms / 0.07f;
    float power = voltageRMS * currentRMS;
    // Serial.printf("Conversiones: %d\n", adc_coversion_count);
    Serial.printf("%.4f, %.4f, %.4f\n", Vrms, currentRMS, power);

    

    consensus_msg_t msg = { power, myPriority };
    esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));

    // 3) esperar ventana para recibir de todos
    delay(WINDOW_MS);

    // // 4) purgar peers inactivos
    purgeStalePeers();

    // // 5) calcular potencia total (suma de todos los nodos)
    float totalPower = power;
    for (auto &p : peers) {
      totalPower += p.power;
    }

    // Reiniciar ciclo de muestreo
    delay(1000);
    analogContinuousStart();

    // 6) mostrar estado
    // Serial.printf(
    //   "nodosActivos: %d  consumoTotal: %.2f W\n", 
    //   peers.size() + 1,     // +1 = este nodo
    //   totalPower
    // );

    // 7) lógica futura: usar p.priority de cada peer para conectar/desconectar cargas

    // 8) esperar antes del próximo ciclo

  }
}