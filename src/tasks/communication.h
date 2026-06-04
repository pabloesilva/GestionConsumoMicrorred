#pragma once

#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <esp_now.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <MCP2515.h>
#include <SPI.h>

// ---------------- ESPNOW disponibilidad ----------------
struct availability_msg_t{
    float availableCurrent;
};

// ---------------- consenso ----------------
struct consensus_msg_t{
    float power;
    uint8_t priority;
};

// ---------------- peer ----------------
struct PeerData{
    uint8_t mac[6];
    float power;
    uint8_t priority;
    unsigned long lastSeen;
    unsigned long secondLastSeen;
};

// broadcast
static constexpr uint8_t broadcastAddress[6] ={
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF 
};

bool comm_Init();
void comm_Process();
void comm_updatePeer(
    const uint8_t* mac,
    float power,
    uint8_t priority);

void comm_PurgeStalePeers();

float comm_GetTotalConsumption();

int comm_GetPeerCount();

std::vector<PeerData>& comm_GetPeers();

PeerData* comm_FindNewestPeer();
    
void comm_processCanRx();

void comm_processCanTx();

void comm_processEspNow();

void comm_processUart();

bool comm_InitCan();

bool comm_InitEspNow();
