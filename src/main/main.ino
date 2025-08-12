#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <driver/gpio.h>
#include <esp_timer.h>           // para temporizador de alta precisión

#define CONVERSIONS_PER_PIN 3

// direccion broadcast para ESP-NOW
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//estructura que llega del nodo informante
typedef struct {
  float availablePower;   // potencia total disponible (W)
} availability_msg_t;

// estructura de mensaje de consenso: potencia + prioridad
typedef struct {
  float power;       // consumo o generación en W
  uint8_t priority;  // 0 = más alta, 3 = más baja
} consensus_msg_t;

//variable para guardar el ultimo consumo para la reconexión
float lastPower = 0;
float totalPower = 0;

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
const unsigned long WINDOW_MS = 6000;

// Pines y configuración
const int sensorPin = 35;                       // GPIO35 -> ADC1_CHANNEL_7
const int Interruptor0 = 26;
const int Interruptor1 = 14;
const int rele = 25;
const int ledRojo1 = 15, ledRojo2 = 4;
const int ledAmarillo1 = 5, ledAmarillo2 = 19;
const int ledVerde1 = 22, ledVerde2 = 23;
const int boton = 33;

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
const float sensibility = 0.07f;
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
adc_continuous_data_t * result = NULL;
// ISR conversión completa ADC
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

// timers para parpadear led ante una disponibilidad menor a la minima para encender 1 led
static esp_timer_handle_t pulsePeriodTimer = nullptr; // periodic: inicia pulso
static esp_timer_handle_t pulseOffTimer = nullptr;    // one-shot: apaga pulso
// parametros globales para los dos pulsos
static uint32_t pulsePeriodMs_global = 1500; // period between pulses
static uint32_t pulseWidthMs_global  = 100;  // pulse width
// callback one-shot: apaga el led (se ejecuta en contexto de timer task)
void IRAM_ATTR onPulseOff(void* arg) {
  gpio_set_level((gpio_num_t)ledRojo1, 0);
}
// callback periodico: enciende el led y arma el one-shot para apagarlo
void IRAM_ATTR onPulsePeriod(void* arg) {
  // encender led inmediatamente
  gpio_set_level((gpio_num_t)ledRojo1, 1);
  // arrancar timer one-shot para apagar
  if (pulseOffTimer) {
    uint64_t off_us = (uint64_t)pulseWidthMs_global * 1000ULL;
    esp_timer_start_once(pulseOffTimer, off_us);
  }
}
// crea timers si no existen aun
void setupPulseTimers() {
  if (pulsePeriodTimer && pulseOffTimer) return;
  const esp_timer_create_args_t off_args = {
    .callback = &onPulseOff,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "pulse_off"
  };
  esp_timer_create(&off_args, &pulseOffTimer);
  const esp_timer_create_args_t period_args = {
    .callback = &onPulsePeriod,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "pulse_period"
  };
  esp_timer_create(&period_args, &pulsePeriodTimer);
}
// inicia parpadeo en modo pulso: period_ms = 1500, pulse_ms = ancho del pulso
void startPulseBlink(uint32_t period_ms = 1500, uint32_t pulse_ms = 100) {
  setupPulseTimers();
  pulsePeriodMs_global = period_ms;
  pulseWidthMs_global  = pulse_ms;
  // asegurar led apagado antes de arrancar
  gpio_set_level((gpio_num_t)ledRojo1, 0);
  // arrancar timer periodico (microsegundos)
  uint64_t period_us = (uint64_t)period_ms * 1000ULL;
  esp_timer_start_periodic(pulsePeriodTimer, period_us);
}
// detiene parpadeo
void stopPulseBlink() {
  if (pulsePeriodTimer) esp_timer_stop(pulsePeriodTimer);
  if (pulseOffTimer) esp_timer_stop(pulseOffTimer);
  // asegurar estado apagado
  gpio_set_level((gpio_num_t)ledRojo1, 0);
}

availability_msg_t msg_disp; //variable global con el mensaje de disponibilidad

// callback de recepción ESP-NOW
void onDataRecv(const esp_now_recv_info_t * info, const uint8_t* buf, int len) {
  
  uint8_t mac[6]; //dirrecion mac (id unica) que identifica a cada esp32
  memcpy(mac, info->src_addr, 6);

  //si el mensaje es de disponibilidad proveniente de la microrred
  if (len == sizeof(availability_msg_t)) {
    memcpy(&msg_disp, buf, len);

    // definimos la franja de cada led
    const float maxPower = 2200.0f;
    const float segment = maxPower / 6.0f;  // ~366.67 W por led

    // calculamos cuantos se deben encender
    int ledsOn = int(msg_disp.availablePower / segment + 0.0001f);
    if (ledsOn > 6) ledsOn = 6;
    if (ledsOn < 0) ledsOn = 0;

    if (ledsOn == 0) {
      // activar modo parpadeo: apagar todos los leds y arrancar pulse blink
      for (int i = 0; i < 6; ++i) digitalWrite(ledPins[i], LOW);
      startPulseBlink(1500, 100); // pulso corto cada 1.5s, ancho 100ms
    } else {
      // desactivar parpadeo si estaba activo y mostrar estado proporcional
      stopPulseBlink();
      for (int i = 0; i < 6; ++i) {
        digitalWrite(ledPins[i], (i < ledsOn) ? HIGH : LOW);
      }
    }
    return;
  }
  
  // si el mensaje no es de concenso
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

// funcion para purgar peers que no hayan enviado en más tiempo que la ventana establecida (WINDOW_MS)
void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = peers.size() - 1; i >= 0; --i) {
    if (now - peers[i].lastSeen > WINDOW_MS) {
      peers.erase(peers.begin() + i);
    }
  }
}

// leee la prioridad a partir del dipswitch (0..3)
// devuelve: 0 = prioridad mas alta, 3 = prioridad mas baja
uint8_t readDipPriority() {
  // lectura raw: LOW = switch ON (activo), HIGH = switch OFF
  bool raw_msb = digitalRead(Interruptor0);
  bool raw_lsb = digitalRead(Interruptor1);

  // invertir porque son activos en bajo
  bool msb_on = !raw_msb;
  bool lsb_on = !raw_lsb;

  uint8_t msb = msb_on ? 1 : 0;
  uint8_t lsb = lsb_on ? 1 : 0;

  uint8_t priority = (msb << 1) | lsb; // msb:bit1, lsb:bit0
  return priority; // valor 0..3
}

uint8_t myPriority; 

void setup() {
  Serial.begin(115200);

  // configurar pines
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
  
  // configurar timers para parpadeo ante baja disponibilidad
  setupPulseTimers();

  // configurar ADC
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, Fs, &adcComplete);
    
  // --- CALIBRACIÓN DE OFFSET ---
  // encender LEDs 
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
  // iniciar la conversión continua del ADC
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
  // parar la conversión continua del ADC
  analogContinuousStop();

  float sum = 0.0f;
  for (int i = 0; i < bufferSize; i++) {
    sum += adcBuffer[i] / 1000.0f;  // convertir a voltios
  }
  voltageOffset = sum / bufferSize;
  bufferFull = false;
  Serial.printf("Offset calibrado: %.5f V\n", voltageOffset);

  // apagar LEDs
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

  myPriority = readDipPriority(); 
  // imprimir en consola una vez la prioridad
  Serial.printf("priority dipswitch: %u\n", myPriority);

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
  digitalWrite(rele,HIGH);

  // iniciar la conversión continua del ADC
  analogContinuousStart();
}


void loop() {
  // prioridad para el nodo, siendo 0 la mas alta y 3 la mas baja
                   

  // primer clausula, chequear si el circuito esta activo consumiendo corriente
  if (digitalRead(rele)){

    // finalizacion de la conversion, se detiene el muestreo continuo
    if (adc_coversion_done){
      adc_coversion_done = false;
      if (analogContinuousRead(&result, 0)){
        adcBuffer[bufferIndex++] = result[0].avg_read_mvolts;
        if (bufferIndex >= bufferSize) {
          analogContinuousStop();
          bufferFull = true;
          bufferIndex = 0;
        }
      }
    }

    // si se lleno el buffer de muestras, empieza el calculo
    if (bufferFull) {  
      bufferFull = false;
      float sumsq = 0.0;
      for (int i = 0; i < bufferSize; i++) {
        float V = adcBuffer[i] / 1000.0f;         // convertir a voltios
        V = V - voltageOffset;                    // restar valor promedio de la señal
        sumsq += V * V;                           // sumatoria de valores al cuadrado 
      }
      float Vrms = sqrt(sumsq / bufferSize);      // promedio sobre cantidad de muestras y raiz para obtener valor eficaz (RMS) discreto 
      
      // Vrms = Vrms - 0.0135;                    // umbral de ruido inherente al sensor
      // Vrms = (Vrms < 0.0005) ? 0 : Vrms;       // ventana de histeresis para valores muy pequeños
      
      float currentRMS = Vrms / sensibility;      // convertir valor en tension a corriente
      float power = voltageRMS * currentRMS;      // calculo de potencia aparente
      
      // mostrar valores calculados  por consola
      Serial.printf("%.4f, %.4f, %.4f\n", Vrms, currentRMS, power);
      // Serial.printf("Valor rele: %d \n", digitalRead(rele));

      // enviar mensaje de consumo a los demas nodos de consumo
      consensus_msg_t msg_send = { power, myPriority };
      esp_now_send(broadcastAddress, (uint8_t*)&msg_send, sizeof(msg_send));

      // 3) esperar ventana para recibir de todos los nodos de consumo
      delay(WINDOW_MS/10);

      // 4) purgar peers inactivos
      purgeStalePeers();

      // 5) calcular potencia total (suma de todos los nodos)
      totalPower = power;
      for (auto &p : peers) {
        totalPower += p.power;
      }

      // 6) mostrar estado por consola 
      Serial.printf(
        "nodosActivos: %d  consumoTotal: %.2f W\n", 
        peers.size() + 1,     // +1 = este nodo
        totalPower
      );

      // 7) conexion/desconexion cargas
      if (totalPower > msg_disp.availablePower) {                   // primero chequear si el consumo supera la potencia disponible
        uint8_t minPriority = 1;                                    // se establece como base la prioridad no critica mas alta
        uint8_t samePriority = 0;                                   // tambien un contador si coincide la criticidad de uno o mas nodos 
        for (auto &p : peers) {                                     // se recorre la lista de peers y se actualiza a cual posee la prioridad minima
          if (p.priority > minPriority) {
            minPriority = p.priority;
            if(p.priority == myPriority){                   
              samePriority++;                                       // se incrementa el contador si existen varios nodos con la misma prioridad 
            }
          }
        }
        
        if (minPriority < myPriority){                              // por ultimo se chequea si la menor prioridad se corresponde a este
          minPriority = myPriority;
        }

        // decidir si este nodo se debe desconectar
        if (myPriority == minPriority) {                            // verificar si la prioridad de este nodo es la menor
          if(samePriority){                                         // si hay más de un nodo con la misma prioridad 
            for (auto &p : peers){
              if(p.priority == myPriority && p.power > power){      // si tiene la misma prioridad y el consumo es el menor
                lastPower = power;                                  // guardar el ultimo consumo para la reconexión
                digitalWrite(rele, LOW);                            // desconectar esta carga
              }
            }
          }
          else{                                                     // si no hay otro nodo con la misma prioridad
            lastPower = power;
            digitalWrite(rele, LOW);                                // desconectar esta carga
          }
        }
      }
      analogContinuousStart();
    }
  }else{
    //se vuelve a calcular la potencia con el ultimo dato de consumo mas un 5%
    totalPower = 1.05*lastPower;
    for (auto &p : peers) {
      totalPower += p.power;
    }
    //para decidir si volverse a conectar o no se verifica si la potencia disponible es suficiente
    if ((msg_disp.availablePower - totalPower) > 0){
      digitalWrite(rele, HIGH);
    }
  }
}
