#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <esp_now.h>
#include <WiFi.h>
#include <vector>
#include <cmath>

#define CONVERSIONS_PER_PIN 3

// --------------------------------------- OLED CONFIG ---------------------------------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// I2C pins (ESP32)
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// --------------------------------------- REDES & MENSAJES ---------------------------------------
// direccion broadcast para ESP-NOW
static uint8_t broadcastAddress[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//estructura que llega del nodo informante
typedef struct {
  float availableCurrent;                           // Corriente total disponible (A)
} availability_msg_t;

uint8_t myPriority; 

// estructura de mensaje de consenso: potencia + prioridad
typedef struct {
  float current;                                    // consumo o generación en A
  uint8_t priority;                                 // 0 = más alta, 3 = más baja
} consensus_msg_t;

//variable para guardar el ultimo consumo para la reconexión
float lastCurrent = 0;
float totalCurrent = 0;

// datos de cada peer, indexados por MAC
struct PeerData {
  uint8_t mac[6];
  float current;
  uint8_t priority;
  unsigned long lastSeen;
};  

// almacenamiento de corrientes recibidas
static std::vector<PeerData> peers;

// ventana de recepción en milisegundos
const unsigned long WINDOW_MS = 5000;

// ------------------------------ GPIO & VARIABLES PRINCIPALES ------------------------------
// Pines y configuración
const int sensorPin = 34;                                     // GPIO34 -> ADC1_CHANNEL_6
const int Interruptor0 = 26;
const int Interruptor1 = 14;
const int rele = 33;

// Constantes de Cálculo
const float sensibility = 0.072f;
const float lineFreq = 50;
const int Fs = 50000;
const int samplesPerPeriod = Fs / lineFreq;
const int periodsToCapture = 10;
const int bufferSize = samplesPerPeriod * periodsToCapture;

// Buffers y flags
static volatile uint16_t adcBuffer[bufferSize];
static volatile int bufferIndex = 0;
static volatile bool bufferFull = false;
float voltageOffset = 0.0f;
bool firstMeasure = true;

// parámetros del ADC continuo
uint8_t adc_pins[] = {sensorPin}; 
uint8_t adc_pins_count = 1;
volatile bool adc_coversion_done = false;
adc_continuous_data_t * result = NULL;

// ISR conversión completa ADC
void ARDUINO_ISR_ATTR adcComplete() {
  adc_coversion_done = true;
}

// ---------- DISPLAY PULSE (controlado desde loop con millis) ----------
volatile bool displayPulseActive = false;
uint32_t displayPulsePeriodMs = 1500;
uint32_t displayPulseWidthMs  = 100;
volatile int displayLedsOn = 0; // cantidad visual en pantalla (0 ... displayDivisions)
const float displayMaxA = 10.0f; // escala 0 ... 10 A
const int displayDivisions = 12; // más divisiones que los 6 LEDs 

// mensajes de disponibilidad actuales
availability_msg_t msg_disp; //variable global con el mensaje de disponibilidad

// variable para mostrar consumo local en la pantalla (actualizada cuando se mide)
float lastMeasuredCurrent = 0.0f;

// ------- pantalla / alternancia -------
unsigned long lastScreenSwitch = 0;
const uint32_t screenSwitchInterval = 4000; // 5 segundos
int screenMode = 0; // 0 = mostrar disponibilidad (barra + valor abajo), 1 = mostrar consumo grande, 2=prioridad

// ----------------- WRAPPERS DE INDICADORES -----------------
// Inicializa display
void indicatorsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Error: SSD1306 no inicializado");
  } else {
    display.clearDisplay();
    display.display();
  }
}

// limpiar display (barra)
void indicatorsClearDisplay() {
  displayLedsOn = 0;
  display.clearDisplay();
  display.display();
}

// iniciar/detener "parpadeo de baja disponibilidad" (solo display)
void startPulseBlink(uint32_t period_ms = 1500, uint32_t pulse_ms = 100) {
  displayPulseActive = true;
  displayPulsePeriodMs = period_ms;
  displayPulseWidthMs = pulse_ms;
}
void stopPulseBlink() {
  displayPulseActive = false;
}

// NUEVA FUNCION: Dibuja barra de progreso
void displayCalibrationProgress(float progress) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Calibrando offset...");
  display.display();
  
  display.fillRect(0, 10, SCREEN_WIDTH, SCREEN_HEIGHT - 10, SSD1306_BLACK);
  int x = 8, y = 12, w = SCREEN_WIDTH - 16, h = 12;
  int innerW = w - 4, innerH = h - 4, startX = x + 2, startY = y + 2;
  display.drawRect(x, y, w, h, SSD1306_WHITE);
  if (progress < 0.0f) progress = 0.0f; 
  if (progress > 1.0f) progress = 1.0f;
  int fillW = int(innerW * progress);
  if (fillW > 0) { 
    display.fillRect(startX, startY, fillW, innerH, SSD1306_WHITE); 
  }
  display.display();
}


// ----------------- DISPLAY DRAW FUNCTION -----------------
// Dibuja la "batería" con escala 0..displayMaxA y las divisiones
// Muestra el valor de disponibilidad debajo de la barra cuando screenMode == 0
// Cuando screenMode == 1 borra la barra y muestra el consumo en fuente grande
void displayUpdate() {
  unsigned long now = millis();
  if (now - lastScreenSwitch >= screenSwitchInterval) {
    lastScreenSwitch = now;
    screenMode = (screenMode + 1) % 3; // Cycle through 3 modes
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  if (screenMode == 0) {
    // Pantalla 0: Disponibilidad con barra interactiva
    display.setTextSize(1);
    display.setCursor(0, 0); 
    display.print("0A");
    display.setCursor(SCREEN_WIDTH - 20, 0); 
    display.print(String(displayMaxA,0) + "A");
    int x=8, y=10, w=SCREEN_WIDTH-16, h=12;
    display.drawRect(x, y, w, h, SSD1306_WHITE);
    int div=displayDivisions, innerW=w-4, innerH=h-4, startX=x+2, startY=y+2;
    
    for (int i = 0; i <= div; ++i) { 
      int dx = startX + (i * innerW) / div; 
      display.drawFastVLine(dx, startY, innerH, SSD1306_WHITE); 
    }
    int fillSegments = displayLedsOn;
    
    if (fillSegments < 0) fillSegments = 0; 
    if (fillSegments > div) fillSegments = div;
    
    bool showFill = true;
    
    if (displayPulseActive) { 
      uint32_t ms = millis() % displayPulsePeriodMs; 
      showFill = (ms < displayPulseWidthMs); 
    }
    
    if (fillSegments > 0 && showFill) { 
      int fillW = (innerW * fillSegments) / div; 
      display.fillRect(startX, startY, fillW, innerH, SSD1306_WHITE); 
    }
    
    else if (!showFill && displayPulseActive) { 
      display.setTextSize(1); 
      display.setCursor(15, startY + 2 ); 
      display.print("! NO DISPONIBLE !"); 
    }

    display.setTextSize(1); 
    int yText = startY + innerH + 3; 
    display.setCursor(48, yText);
    display.print(String(msg_disp.availableCurrent, 2)); 
    display.print(" A");

  } else if (screenMode == 1) {
    // Pantalla 1: Consumo Local
    display.setTextSize(1); 
    display.setCursor(0, 0); 
    display.print("Consumo:");
    display.setTextSize(2); 
    String s = String(lastMeasuredCurrent, 2) + " A";
    int16_t x1, y1; uint16_t w1, h1; 
    display.getTextBounds(s, 0, 0, &x1, &y1, &w1, &h1);
    int xText = (SCREEN_WIDTH - w1) / 2; 
    int yText = 15; 
    if (xText < 0) xText = 0;
    display.setCursor(xText, yText); 
    display.print(s);
    } 
    else { 
    // Mostrar el número de prioridad en grande (centrado)
    display.setTextSize(2); // Tamaño grande para el número
    String prioNumStr = "P"+ String(myPriority);
    int16_t x1, y1; uint16_t w1, h1;
    display.getTextBounds(prioNumStr, 0, 0, &x1, &y1, &w1, &h1); // Medir tamaño
    // Centrar horizontalmente, posicionar verticalmente en el medio
    int xNum = (SCREEN_WIDTH - w1) / 2;
    int yNum = 1; // Ajustar posición vertical si es necesario
    if (xNum < 0) xNum = 0;
    display.setCursor(xNum, yNum);
    display.print(prioNumStr);

    // Mostrar descripción debajo del número (en tamaño pequeño)
    display.setTextSize(1);
    String priorityDesc = "";
    bool isCritical = (myPriority == 0);

    if (isCritical) {
        priorityDesc = "CRITICA";
    } else {
        priorityDesc = "NO CRITICA - ";
        switch (myPriority) {
            case 1: priorityDesc += "ALTA"; break;
            case 2: priorityDesc += "MEDIA"; break;
            case 3: priorityDesc += "BAJA"; break;
            default: priorityDesc = "Prioridad INVALIDA"; break; // Caso de error
        }
    }

    // Centrar descripción horizontalmente, debajo del número
    display.getTextBounds(priorityDesc, 0, 0, &x1, &y1, &w1, &h1);
    int xDesc = (SCREEN_WIDTH - w1) / 2;
    int yDesc = 21; // Posición Y en la última línea
    if (xDesc < 0) xDesc = 0;
    display.setCursor(xDesc, yDesc);
    display.print(priorityDesc);
  }
  // --- Llamada única a display() al final ---
  display.display();
}

// ----------------- CALLBACK ESP-NOW -----------------
void onDataRecv(const esp_now_recv_info_t * info, const uint8_t* buf, int len) {
  uint8_t mac[6]; //dirrecion mac (id unica) que identifica a cada esp32
  memcpy(mac, info->src_addr, 6);

  //si el mensaje es de disponibilidad proveniente de la microrred
  if (len == sizeof(availability_msg_t)) {
    memcpy(&msg_disp, buf, len);

    // definimos la franja para la pantalla (usamos displayDivisions)
    const float maxCurrent = displayMaxA; // 10.0
    const float segment = maxCurrent / float(displayDivisions);

    // calculamos cuantos segmentos se deben encender en la pantalla
    int ledsOn = int(msg_disp.availableCurrent / segment + 0.0001f);
    if (ledsOn > displayDivisions) ledsOn = displayDivisions;
    if (ledsOn < 0) ledsOn = 0;

    if (ledsOn == 0) {
      // activar modo parpadeo en display
      indicatorsClearDisplay();
      startPulseBlink(displayPulsePeriodMs, displayPulseWidthMs);
    } else {
      // desactivar parpadeo si estaba activo y mostrar estado proporcional
      stopPulseBlink();
      displayLedsOn = ledsOn;
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
      p.current    = msg.current;
      p.priority = msg.priority;
      p.lastSeen = millis();
      return;
    }
  }
  // si no existe, agregar nuevo
  PeerData np;
  memcpy(np.mac, mac, 6);
  np.current    = msg.current;
  np.priority = msg.priority;
  np.lastSeen = millis();
  peers.push_back(np);
}

// funcion para purgar peers que no hayan enviado en más tiempo que la ventana establecida (WINDOW_MS)
void purgeStalePeers() {
  unsigned long now = millis();
  for (int i = (int)peers.size() - 1; i >= 0; --i) {
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


void setup() {
  Serial.begin(115200);

  // config pines sensor y switches
  pinMode(sensorPin, INPUT);
  pinMode(Interruptor0, INPUT);
  pinMode(Interruptor1, INPUT);
  pinMode(rele, OUTPUT);

  // inicializo indicadores (solo display)
  indicatorsInit();

  // configurar ADC
  analogContinuousSetWidth(12);
  analogContinuousSetAtten(ADC_11db);
  analogContinuous(adc_pins, adc_pins_count, CONVERSIONS_PER_PIN, Fs, &adcComplete);
    
  // ----------------------------- CALIBRACIÓN DE OFFSET -----------------------------

  //Serial.println("Iniciando calibración de offset...");
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

  // ------------------------ bucle para la animación de inicio ------------------------
  unsigned long calAnimStartTime = millis();
  const unsigned long calibrationAnimationDuration = 1000; // 1 segundo
  unsigned long lastProgressUpdateAnim = 0;
  const unsigned long progressUpdateIntervalAnim = 50; // ms

  while (millis() - calAnimStartTime < calibrationAnimationDuration) {
      unsigned long nowAnim = millis();
      if (nowAnim - lastProgressUpdateAnim >= progressUpdateIntervalAnim) {
          lastProgressUpdateAnim = nowAnim;
          float currentProgress = (float)(nowAnim - calAnimStartTime) / (float)calibrationAnimationDuration;
          displayCalibrationProgress(currentProgress);
      }
  }
  displayCalibrationProgress(1.0f); // Asegurar 100% al final
  //delay(300); // Pausa corta

  float sum = 0.0f;
  for (int i = 0; i < bufferSize; i++) {
    sum += adcBuffer[i] / 1000.0f;  // convertir a voltios
  }
  voltageOffset = sum / bufferSize;
  bufferFull = false;
  //Serial.printf("Offset calibrado: %.5f V\n", voltageOffset);

  myPriority = readDipPriority(); 
  //Serial.printf("priority dipswitch: %u\n", myPriority);

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

  // inicializar pantallas/variables
  lastMeasuredCurrent = 0.0f;
  lastScreenSwitch = millis();
  screenMode = 0;
}


void loop() {

  // primer clausula, chequear si el circuito esta activo consumiendo corriente
  if (digitalRead(rele)){
      if (firstMeasure){
        delay(1000);
        firstMeasure = false;
        analogContinuousStart();
    }

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
      // 1) conversion de valores digitales a mV
      bufferFull = false;
      float sumsq = 0.0;
      for (int i = 0; i < bufferSize; i++) {
        float V = adcBuffer[i] / 1000.0f;         // convertir a voltios
        V = V - voltageOffset;                    // restar valor promedio de la señal
        sumsq += V * V;                           // sumatoria de valores al cuadrado 
      }
      float Vrms = sqrt(sumsq / bufferSize);      // promedio sobre cantidad de muestras y raiz para obtener valor eficaz (RMS) discreto 
      
      // Vrms = Vrms - 0.0135;                    // umbral de ruido inherente al sensor
      Vrms = (Vrms < 0.018) ? 0 : Vrms;           // ventana de histeresis de ruido en la medicion
      
      float currentRMS = Vrms / sensibility;      // convertir valor en tension a corriente
      
      lastMeasuredCurrent = currentRMS;           // guardar para display

      // mostrar valores calculados  por consola
      Serial.printf("Vrms: %.4f, Arms: %.4f\n", Vrms, currentRMS);

      // 2) enviar mensaje de consumo a los demas nodos de consumo
      consensus_msg_t msg_send = { currentRMS, myPriority };
      esp_now_send(broadcastAddress, (uint8_t*)&msg_send, sizeof(msg_send));

      // 3) esperar ventana para recibir de todos los nodos de consumo
      delay(WINDOW_MS/10);

      // 4) purgar peers inactivos
      purgeStalePeers();

      // 5) calcular potencia total (suma de todos los nodos)
      totalCurrent = currentRMS;
      for (auto &p : peers) {
        totalCurrent += p.current;
      }

      // 6) mostrar estado por consola 
      // Serial.printf(
      //   "nodosActivos: %d  consumoTotal: %.2f A\n", 
      //   peers.size() + 1,     // +1 = este nodo
      //   totalCurrent
      // );


      // 7) logica de conexion/desconexion cargas
      if(!(myPriority == 0)){
        
        if (totalCurrent > msg_disp.availableCurrent) {                   // primero chequear si el consumo supera la potencia disponible
          uint8_t minPriority = 1;                                        // se establece como base la prioridad no critica mas alta
          uint8_t samePriority = 0;                                       // tambien un contador si coincide la criticidad de uno o mas nodos 
          for (auto &p : peers) {                                         // se recorre la lista de peers y se actualiza a cual posee la prioridad minima
            if (p.priority > minPriority) {
              minPriority = p.priority;
              if(p.priority == myPriority){                   
                samePriority++;                                           // se incrementa el contador si existen varios nodos con la misma prioridad 
              }
            }
          }
          
          if (minPriority < myPriority){                                  // por ultimo se chequea si la menor prioridad se corresponde a este
            minPriority = myPriority;
          }

          // decidir si este nodo se debe desconectar
          if (myPriority == minPriority) {                                // verificar si la prioridad de este nodo es la menor
            if(samePriority){                                             // si hay más de un nodo con la misma prioridad 
              for (auto &p : peers){
                if(p.priority == myPriority && p.current > currentRMS){   // si tiene la misma prioridad y el consumo es el menor
                  lastCurrent = currentRMS;                               // guardar el ultimo consumo para la reconexión
                  digitalWrite(rele, LOW);                                // desconectar esta carga
                }
              }
            }
            else{                                                         // si no hay otro nodo con la misma prioridad
              lastCurrent = currentRMS;
              digitalWrite(rele, LOW);                                    // desconectar esta carga
            }
          }
        }
      }
      analogContinuousStart();
    }
  }else{
    // 4) purgar peers inactivos
    purgeStalePeers();
    firstMeasure = true;
    //se vuelve a calcular la potencia con el ultimo dato de consumo mas un 5%
    totalCurrent = 1.05*lastCurrent;
    for (auto &p : peers) {
      totalCurrent += p.current;
    }
    //para decidir si volverse a conectar o no se verifica si la potencia disponible es suficiente
    if ((msg_disp.availableCurrent - totalCurrent) > 0){
      digitalWrite(rele, HIGH);
    }
  }

  //actualización periódica de display
  displayUpdate();
}
