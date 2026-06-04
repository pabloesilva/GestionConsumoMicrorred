#pragma once

#include <Arduino.h>

bool display_Init();

void display_Process();

void display_RequestUpdate();

void display_ForceRedraw();

void display_Update();