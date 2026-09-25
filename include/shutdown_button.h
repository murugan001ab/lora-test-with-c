// include/shutdown_button.h
//
// Graceful shutdown: hold the BOOT button (active LOW) for
// SHUTDOWN_HOLD_MS to send a device_offline status and enter deep sleep.

#pragma once
#include <Arduino.h>

// Sets pinMode for the shutdown button. Call once from setup().
void initShutdownButton();

// Non-blocking — call every loop() iteration.
void checkShutdownButton();
