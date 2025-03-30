#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#define MAX_PEERS 10  

uint8_t peers[MAX_PEERS][6];  
int peerCount = 0;

typedef struct {
  float consumoActual;
  int prioridad;
} EnergiaData;

EnergiaData energia;
const int prioridadNodo = 2;  
bool cargaEncendida = true;
bool inicioCom = true;

uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

const int sensorPin = 34;                   // Pin ADC donde se conecta la salida del ACS712

const int Interruptor = 14;   
const int rele = 25 ;  

const float voltageRMS = 220.0;             // Voltaje RMS de la red eléctrica
const float sensitivity = 0.069 ;           // Sensibilidad del ACS712 (100 mV/A para el modelo de 20A)
const float adcResolution = 3.3 / 4096.0;   // Resolución del ADC del ESP32 (3.3V referencia / 4095 pasos)
const float voltageOffset = 1.59;           // Voltaje de offset del sensor después del divisor resistivo (1.65V)

const int numSamples = 8000;                // Número de muestras totales 

float power = 0;
float powerKm1 = 0;
float powerKm2 = 0;
float powerKm3 = 0;
float powerMEDIAMOVIL = 0;

void agregarPeer(const uint8_t *mac) {
  if (peerCount >= MAX_PEERS) return;  

  for (int i = 0; i < peerCount; i++) {
    if (memcmp(peers[i], mac, 6) == 0) return;  // Ya está registrado
  }

  memcpy(peers[peerCount], mac, 6);
  peerCount++;

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) Serial.println("Peer agregado correctamente");
  else Serial.println("Error al agregar peer");
}

// Función para enviar información a todos los peers
void enviarInformacion() {
  if(inicioCom){
    esp_now_send(broadcastAddress, (uint8_t *)&energia, sizeof(energia));
    inicioCom = false;
  }
  else{
    for (int i = 0; i < peerCount; i++) {
      esp_now_send(peers[i], (uint8_t *)&energia, sizeof(energia));
    }
  }
}

// Callback de recepción
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(float)) { // MENSAJE DEL MAESTRO
      float energiaDisponible;
      memcpy(&energiaDisponible, data, sizeof(float));
      Serial.println("\n\n\n");
      Serial.printf("MENSAJE DEL MAESTRO ");
      for (int i = 0; i < 6; i++) Serial.printf("%02X:", mac[i]);
      Serial.println();
      Serial.printf("Energía disponible: %.2f W\n", energiaDisponible);

      agregarPeer(mac);  // REGISTRAR MAESTRO COMO PEER

      // Actualizar información
      energia.consumoActual = random(10, 50) / 10.0;
      energia.prioridad = prioridadNodo;

      // Ajustar estado de carga
      if (energiaDisponible < 20.0 && prioridadNodo == 3) cargaEncendida = false;
      else if (energiaDisponible < 10.0 && prioridadNodo == 2) cargaEncendida = false;
      else cargaEncendida = true;

      Serial.println(cargaEncendida ? "Carga encendida" : "Carga apagada");
      enviarInformacion();  
  }
  else { // MENSAJE DE OTRO ESCLAVO
      EnergiaData mensaje;
      memcpy(&mensaje, data, sizeof(mensaje));

      Serial.printf("MENSAJE DEL ESCLAVO ");
      for (int i = 0; i < 6; i++) Serial.printf("%02X:", mac[i]);
      Serial.println();
      Serial.printf("Consumo: %.2f W, Prioridad: %d\n", mensaje.consumoActual, mensaje.prioridad);

      agregarPeer(mac);  // REGISTRAR OTRO ESCLAVO COMO PEER
  }
}

/*
// Callback de envío
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Envío a %02X:%02X:%02X:%02X:%02X:%02X - %s\n",
                mac_addr[0], mac_addr[1], mac_addr[2],
                mac_addr[3], mac_addr[4], mac_addr[5],
                status == ESP_NOW_SEND_SUCCESS ? "Éxito" : "Fallo");
}
*/

void setup() {
  
  Serial.begin(115200);
  analogReadResolution(12);                 // Configurar resolución del ADC a 12 bits

  pinMode(Interruptor, INPUT);              // Configurar entrada
  pinMode(rele, OUTPUT);                    // Configurar salida

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  //esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al agregar el peer de broadcast");
  }

}

void loop() {
 
  int estado = digitalRead(Interruptor);                                          // activar circulacion en la carga
  (estado == HIGH) ? digitalWrite(rele, LOW) : digitalWrite(rele, HIGH);
  
  float sumSquared = 0;
  int sensorValue = 0;
  float voltage = 0;
  float voltageDifference = 0;

  if (estado == HIGH){
    for (int i = 0; i < numSamples; i++) {
      sensorValue = analogRead(sensorPin);                                        // Leer valor del ADC
      voltage = sensorValue * adcResolution;                                      // Convertir a voltaje
      //Serial.println(voltage);                                                  // calibracion
      voltageDifference = voltage - voltageOffset;                                // Restar el offset
      voltageDifference = (voltageDifference <= 0.015) ?  0 : voltageDifference;  // ventana de histeresis
      sumSquared += voltageDifference * voltageDifference;                        // Acumular el cuadrado de la diferencia
    }
  }
  
  // Calcular el valor cuadrático medio (RMS) del voltaje
  float meanSquare = sumSquared / numSamples;
  float voltageRMS_ACS712 = sqrt(meanSquare);

  // Convertir el voltaje RMS a corriente RMS
  float currentRMS = (voltageRMS_ACS712) / sensitivity;

  // Calcular la potencia aparente
  power = voltageRMS * currentRMS;

  powerMEDIAMOVIL = (power + powerKm1 + powerKm2 + powerKm3)/4;
  powerKm1 = power;
  powerKm2 = powerKm1;
  powerKm3 = powerKm2;

  int power1 = powerMEDIAMOVIL;

  // Mostrar los resultados en el monitor serie
  Serial.print("\n\n\nVoltaje RMS: ");
  Serial.print(voltageRMS_ACS712, 4);
  Serial.print("\nCorriente RMS: ");
  Serial.print(currentRMS, 3);
  Serial.print("\nPotencia: ");
  Serial.print(power1);
  Serial.print(" W");

}
