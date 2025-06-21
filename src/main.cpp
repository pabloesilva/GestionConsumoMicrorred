#include <Arduino.h>
#include <driver/adc.h>
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

// almacenamiento de potencias recibidas
static std::vector<PeerData> peers;

// ventana de recepción en milisegundos
const unsigned long WINDOW_MS = 200;

// Pines y configuración
const int sensorPin = 34;                       // GPIO34 -> ADC1_CHANNEL_6
const int Interruptor = 14;
const int rele = 26;
const int ledRojo1 = 23, ledRojo2 = 22;
const int ledAmarillo1 = 21, ledAmarillo2 = 19;
const int ledVerde1 = 18, ledVerde2 = 5;

// Constantes de cálculo
const float voltageRMS = 220.0f;
const float adcResolution = 3.3f / 4095.0f;

// Parámetros de muestreo
const float lineFreq = 50;
const int Fs = 10000;
const int samplesPerPeriod = Fs / lineFreq;
const int periodsToCapture = 50;
const int bufferSize = samplesPerPeriod * periodsToCapture;

// Buffers y flags
static volatile uint16_t adcBuffer[bufferSize];
static volatile int bufferIndex = 0;
static volatile bool bufferFull = false;
static float voltageOffset = 0.0f;

// Temporizador hardware
hw_timer_t* samplingTimer = nullptr;

// ---------------------------------------------
// ISR de muestreo: lectura rápida de ADC y buffer
void IRAM_ATTR onTimerCallback() {
  if (!bufferFull && bufferIndex < bufferSize) {
    adcBuffer[bufferIndex++] = adc1_get_raw(ADC1_CHANNEL_6);
    if (bufferIndex >= bufferSize) {
      bufferFull = true;
    }
  }
}

// Inicia muestreo periódico
auto startSampling = []() {
  bufferIndex = 0;
  bufferFull = false;
  timerAlarmDisable(samplingTimer);

  // This is where we indicate the frequency of the interrupts.
  // The value "1000000UL / Fs" (because of the prescaler we set in timerBegin) will produce
  // one interrupt every 100 us (10kHz).
  // The 3rd parameter is true so that the counter reloads when it fires an interrupt, and so we
  // can get periodic interrupts (instead of a single interrupt).
  timerAlarmWrite(samplingTimer, 1000000UL / Fs, true);

  timerAlarmEnable(samplingTimer);
};

// Detiene muestreo
inline void stopSampling() {
  timerAlarmDisable(samplingTimer);
}

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
    if (now - peers[i].lastSeen > 10*WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

void setup() {
  Serial.begin(115200);

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

  // Configurar pines
  pinMode(sensorPin, INPUT);
  pinMode(Interruptor, INPUT);
  pinMode(rele, OUTPUT);
  pinMode(ledRojo1, OUTPUT);
  pinMode(ledRojo2, OUTPUT);
  pinMode(ledAmarillo1, OUTPUT);
  pinMode(ledAmarillo2, OUTPUT);
  pinMode(ledVerde1, OUTPUT);
  pinMode(ledVerde2, OUTPUT);

  // Configurar ADC
  //https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html#analogsetattenuation 
  analogSetPinAttenuation(sensorPin, ADC_11db);
  // this function is used to set the hardware sample bits and read resolution. Default is 12 bits (0 - 4095). Range is 9 - 12.
  adc1_config_width(ADC_WIDTH_BIT_12);
  // ADC_ATTEN_DB_12-> 150 mV ~ 3100 mV
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_12);

  // Initilise the timer.
  // Parameter 1 is the timer we want to use. Valid: 0, 1, 2, 3 (total 4 timers)
  // Parameter 2 is the prescaler. The ESP32 default clock is at 80MhZ. The value "80" will
  // divide the clock by 80, giving us 1,000,000 ticks per second.
  // Parameter 3 is true means this counter will count up, instead of down (false).
  samplingTimer = timerBegin(0, 80, true);

  // Attach the timer to the interrupt service routine named "onTimerCallback".
  // The 3rd parameter is set to "true" to indicate that we want to use the "edge" type (instead of "flat").
  timerAttachInterrupt(samplingTimer, onTimerCallback, true);

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
  startSampling();
  while (!bufferFull) { }
  stopSampling();

  float sum = 0.0f;
  for (int i = 0; i < bufferIndex; i++) {
    sum += adcBuffer[i] * adcResolution;
  }
  voltageOffset = sum / bufferIndex;
  voltageOffset=roundf(voltageOffset * 10000.0f) / 10000.0f;   // Redondear a 4 decimales
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


  startSampling();   // Iniciar muestreo continuo
}

void loop() {
  int estado = digitalRead(Interruptor);
  digitalWrite(rele, estado == HIGH ? LOW : HIGH);

  uint8_t myPriority = 0;                     // ejemplo: prioridad alta


  if (bufferFull) {
    float sumsq = 0.0f;
    for (int i = 0; i < bufferIndex; i++) {
      float V = adcBuffer[i] * adcResolution;
      V = roundf(V * 10000.0f) / 10000.0f;       // Redondear a 4 decimales
      V = V - voltageOffset;
      sumsq += V * V;
    }
    float Vrms = sqrt(sumsq / bufferIndex);
    Vrms = roundf(Vrms * 10000.0f) / 10000.0f;
    Vrms = Vrms - 0.015;
    Vrms=(Vrms<0.0005)?0:Vrms;
    float currentRMS = Vrms / 0.07f;
    currentRMS = roundf(currentRMS * 10000.0f) / 10000.0f;
    float power = voltageRMS * currentRMS;

    Serial.printf("%.4f, %.4f, %.4f\n", Vrms, currentRMS, power);

    // Reiniciar ciclo de muestreo
    stopSampling();
    startSampling();

    consensus_msg_t msg = { power, myPriority };
    esp_now_send(broadcastAddress, (uint8_t*)&msg, sizeof(msg));

    // 3) esperar ventana para recibir de todos
    delay(WINDOW_MS);

    // 4) purgar peers inactivos
    purgeStalePeers();

    // 5) calcular potencia total (suma de todos los nodos)
    float totalPower = power;
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
}

