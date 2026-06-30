#pragma once

// ---------------- OLED ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

// ---------------- CAN ----------------
#define CAN_CS_PIN 5
#define CAN_INT_PIN 4

#define MAX_CAN_RETRIES 3
#define CAN_RESET_THRESHOLD 5
#define CAN_RETRY_INTERVAL_MS 5000
// Reset forzado si no hay ningún envío exitoso en este tiempo (cubre el caso
// donde un éxito esporádico impide que canConsecFailures llegue al umbral).
#define CAN_FORCE_RESET_MS 30000

// ---------------- BOTONES ----------------
#define BTN_PREV_PIN 14
#define BTN_NEXT_PIN 27
#define BTN_HOME_PIN 26

const unsigned long debounceMs = 15;
const unsigned long longPressMs = 3000;

// ---------------- UART ----------------
#define GW_TX_PIN 1
#define GW_RX_PIN 3

const unsigned long uartSendInterval = 500;

// ---------------- PANEL ----------------
#define V_MAX 100.0f
#define I_MAX 50.0f

const float V_MIN = 30.0f;

const unsigned long DATA_EXPIRY_MS = 10000;

// ---------------- PEERS ----------------
const unsigned long WINDOW_MS = 600;


// ---------------- ADC ----------------
const int sensorPin = 33;

const int Fs = 50000;
const int lineFreq = 50;

const int samplesPerPeriod = Fs / lineFreq;
const int periodsToCapture = 10;
const int bufferSize = samplesPerPeriod * periodsToCapture;

// ---------------- CAN TX ----------------
const unsigned long canInterval = 5000;

// ---------------- ESPNOW ----------------
const unsigned long availabilityInterval = WINDOW_MS;