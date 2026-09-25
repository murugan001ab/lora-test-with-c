// src/main.cpp
//
// ZWELDAQ ESP32 firmware entry point — wires together the modules below.
// RFID login/logout + ADS1115/analog sensor sampling + DWIN display +
// SX1262 LoRa uplink/downlink. Behavior matches the original monolithic
// zweldaq.cpp (preserved at legacy/zweldaq_monolithic.cpp); this file
// only reorganizes it into reusable, single-purpose modules.
//
// See legacy/p2p_main.cpp for the earlier raw point-to-point LoRa test
// sketch this replaced as the active main.cpp.

#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "state.h"
#include "time_utils.h"
#include "dwin_display.h"
#include "sensors.h"
#include "radio_sx1262.h"
#include "payloads.h"
#include "rfid_handler.h"
#include "shutdown_button.h"

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println();
  Serial.println("========================================");
  Serial.println("          ZWELDAQ ESP32");
  Serial.println(" RFID + ADS1115 + DWIN + SX1262");
  Serial.println("========================================");

  // epochBase starts at 0 (1970-01-01) until the server sends a
  // TIME_SYNC downlink after the device_connect status is received;
  // see radio_sx1262.cpp's handleDownlink(). Timestamps before that
  // sync will read as 1970 epoch time.
  setEpoch(0);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  initShutdownButton();
  RFID_Init();
  initDwinDisplay();

  Wire.begin();
  initSensors();

  initRadio();
  sendDeviceOnline();

  digitalWrite(LED_PIN, HIGH);

  Serial.println();
  Serial.println("========================================");
  Serial.println("             SYSTEM READY");
  Serial.println("========================================");
  Serial.println("RFID  -> VSPI");
  Serial.println("LoRa  -> HSPI");
  Serial.println("ADS1115 -> I2C");
  Serial.println("DWIN -> UART2");
  Serial.println("========================================");
}

void loop()
{
  handleRFID();
  checkAndProcessDownlink();
  readSensors();

  // ------------------------------------------------------------------
  // WELDING STATE MACHINE
  // ------------------------------------------------------------------

  if (Cal_Current > WELD_CURRENT_THRESHOLD && loggedIn && !weldingStarted)
  {
    weldingStarted   = true;
    lastWeldDataTime = millis();

    sendWeldStart();
    Serial.println("[SYSTEM] WELDING STARTED");
  }

  if (Cal_Current > WELD_CURRENT_THRESHOLD && loggedIn && weldingStarted)
  {
    if (millis() - lastWeldDataTime >= WELD_DATA_INTERVAL)
    {
      lastWeldDataTime = millis();
      sendWeldData();
    }
  }

  if (Cal_Current <= WELD_CURRENT_THRESHOLD && weldingStarted)
  {
    sendWeldStop();
    weldingStarted = false;

    Serial.println("[SYSTEM] WELDING STOPPED");
  }

  checkShutdownButton();

  delay(100);
}
