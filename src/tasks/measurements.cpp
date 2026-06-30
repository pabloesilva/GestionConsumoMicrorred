#include "measurement.h"

#include <Arduino.h>
#include <cstring>
#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "../core/config.h"
#include "../core/globals.h"

const adc1_channel_t adcChannel = ADC1_CHANNEL_5;
const adc_atten_t adcAtten = ADC_ATTEN_DB_12;
const adc_bits_width_t adcWidth = ADC_WIDTH_BIT_12;

static volatile uint16_t adcBuffer[bufferSize];
static volatile int bufferIndex = 0;
static volatile bool bufferFull = false;

static float voltageOffset = 0.0f;

hw_timer_t* samplingTimer = nullptr;

esp_adc_cal_characteristics_t adc_chars;


static void IRAM_ATTR onTimerCallback() {
  if (!bufferFull && bufferIndex < bufferSize) {
    int raw = adc1_get_raw(adcChannel);
    adcBuffer[bufferIndex] = (uint16_t)raw;
    bufferIndex = bufferIndex + 1;
    if (bufferIndex >= bufferSize) {
      bufferFull = true;
    }
  }
}

static void startSampling() {
  bufferIndex = 0;
  bufferFull = false;
  timerAlarmDisable(samplingTimer);
  timerAlarmWrite(samplingTimer, 1000000UL / Fs, true);
  timerAlarmEnable(samplingTimer);
}

static void stopSampling() {
  timerAlarmDisable(samplingTimer);
}

bool Measurement_Init(){
    adc1_config_width(adcWidth);
    adc1_config_channel_atten(adcChannel, adcAtten);
    esp_adc_cal_characterize(ADC_UNIT_1, adcAtten, adcWidth, 1100, &adc_chars);

    // configure timer for ADC sampling (prescaler 80 -> 1 MHz ticks)
    samplingTimer = timerBegin(0, 80, true);
    timerAttachInterrupt(samplingTimer, &onTimerCallback, true);

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
    return true;
}
void Measurement_Process(){
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
}