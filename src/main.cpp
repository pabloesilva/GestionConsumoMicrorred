#include <Arduino.h>
#include "core/config.h"
#include "core/globals.h"
#include "tasks/communication.h"
#include "tasks/display.h" 
#include "tasks/measurement.h"

void setup() {
  //Serial.begin(115200);
  Measurement_Init();

  comm_Init();

  if (!display_Init())
  display_Update();
}

void loop() {
  Measurement_Process();

  comm_Process();

  display_Process();
}
