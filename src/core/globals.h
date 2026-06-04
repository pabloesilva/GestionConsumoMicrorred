#pragma once

#include <Arduino.h>

// ---------------- Display ----------------

extern bool displayNeedsUpdate;

extern volatile bool forceDisplayRedraw;

// ---------------- Potencia ----------------

extern double Vrms;

extern float availablePower;

extern bool panelDataValid;

// ---------------- CAN ----------------

extern bool canInitOk;

extern int canConsecFailures;

extern unsigned long lastCanSentTime;

extern unsigned long secondLastCanSentTime;

// ---------------- ESP-NOW ----------------

extern unsigned long lastMessageTime;

extern unsigned long secondLastMessageTime;
