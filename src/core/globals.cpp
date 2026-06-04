#include "globals.h"

// ---------------- Display ----------------
bool displayNeedsUpdate = true;
volatile bool forceDisplayRedraw = false;

// ---------------- Potencia ----------------
double Vrms = 0;

float availablePower = -1.0f;
bool panelDataValid = false;


// ---------------- CAN ----------------
bool canInitOk = false;
int canConsecFailures = 0;
unsigned long lastCanSentTime = 0;
unsigned long secondLastCanSentTime = 0;

// ---------------- ESP-NOW ----------------
unsigned long lastMessageTime = 0;
unsigned long secondLastMessageTime = 0;

// ---------------- Timers ----------------
unsigned long lastAvailabilitySend = 0;
unsigned long lastCanAttemptMillis = 0;