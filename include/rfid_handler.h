// include/rfid_handler.h
//
// MFRC522 RFID login/logout. Uses the global SPI object as ESP32 VSPI
// (the MFRC522 library doesn't support a separate SPIClass instance).

#pragma once
#include <Arduino.h>

// Configures VSPI with the RFID pins and initializes the MFRC522.
// Call once from setup().
void RFID_Init();

// Non-blocking — call every loop() iteration. Debounces card reads
// (RFID_DEBOUNCE in config.h), and on a new card either logs an operator
// in (sends welderlogin, starts a session) or out (sends welderlogout),
// refusing logout while a weld session is active. Updates the shared
// state in state.h.
void handleRFID();
