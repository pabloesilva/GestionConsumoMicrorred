#include "esp_timer.h"
#include <Arduino.h>
// Pines
const int sensorPin = 34;  // GPIO36 - ADC1_CH0 (ruido)

const float voltageRMS = 220.0;             // Voltaje RMS de la red eléctrica
const float sensitivity = 0.069 ;           // Sensibilidad del ACS712 (100 mV/A para el modelo de 20A)
const float adcResolution = 3.3 / 4096.0;   // Resolución del ADC del ESP32 (3.3V referencia / 4095 pasos)
const float voltageOffset = 1.22;           // Voltaje de offset del sensor después del divisor resistivo (1.65V)

float power = 0;
// Variables LMS
volatile int sensorValue = 0;

volatile bool AdcReady = false;
const int numSamples = 2000;  // Número de muestras totales


// Timer callback: se ejecuta cada T_us µs
void IRAM_ATTR onTimerCallback(void* arg) {
  sensorValue = analogRead(sensorPin);
  AdcReady = true;
}

void setup() {
  analogReadResolution(12); // 12 bits (0-4095)
  // Crear temporizador con esp_timer
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimerCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "sampling_timer"
  };

  esp_timer_handle_t periodic_timer;
  esp_timer_create(&timer_args, &periodic_timer);

  // Frecuencia de muestreo: 2000 Hz (cada 500 µs)
  const int T_us = 500;
  esp_timer_start_periodic(periodic_timer, T_us);

  int numSamples = 0;
}

void loop() {
  int samples = 0;
  float sumSquared = 0;
  int sensorValue = 0;
  float voltage = 0;
  float voltageDifference = 0;
  while (samples < numSamples) {
    if (AdcReady) {
      AdcReady = false;
      samples ++;
      voltage = sensorValue*adcResolution; 
      voltageDifference = voltage - voltageOffset;  
      sumSquared += voltageDifference * voltageDifference;   
    }
  }
  // Calcular el valor cuadrático medio (RMS) del voltaje
  float meanSquare = sumSquared / numSamples;
  float voltageRMS_ACS712 = sqrt(meanSquare);

  // Convertir el voltaje RMS a corriente RMS
  float currentRMS = (voltageRMS_ACS712) / sensitivity;

  // Calcular la potencia aparente
  power = voltageRMS * currentRMS;

  // Mostrar los resultados en el monitor serie
  Serial.print("\n\n\nVoltaje RMS: ");
  Serial.print(voltageRMS_ACS712, 4);
  Serial.print("\nCorriente RMS: ");
  Serial.print(currentRMS, 3);
  Serial.print("\nPotencia: ");
  Serial.print(power);
  Serial.print(" W");

}