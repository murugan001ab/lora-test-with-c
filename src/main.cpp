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
#include <esp_system.h>

#include "config.h"
#include "state.h"
#include "time_utils.h"
#include "dwin_display.h"
#include "sensors.h"
#include "radio_sx1262.h"
#include "payloads.h"
#include "rfid_handler.h"
#include "shutdown_button.h"

static void printResetReason()
{
  // Prints WHY the ESP32 just booted -- lets us tell a normal
  // power-on/flash apart from a brownout (power supply sagging under
  // load, e.g. LoRa TX current draw) or a watchdog timeout (something
  // in loop() blocked too long) the next time this happens, instead of
  // guessing from a garbled ROM banner alone.
  esp_reset_reason_t reason = esp_reset_reason();
  const char *reasonStr;

  switch (reason)
  {
    case ESP_RST_POWERON:   reasonStr = "POWERON (normal power-up)"; break;
    case ESP_RST_EXT:       reasonStr = "EXT (external reset pin)"; break;
    case ESP_RST_SW:        reasonStr = "SW (esp_restart() called)"; break;
    case ESP_RST_PANIC:     reasonStr = "PANIC (exception/crash!)"; break;
    case ESP_RST_INT_WDT:   reasonStr = "INT_WDT (interrupt watchdog -- code blocked too long in an ISR)"; break;
    case ESP_RST_TASK_WDT:  reasonStr = "TASK_WDT (task watchdog -- loop() blocked too long)"; break;
    case ESP_RST_WDT:       reasonStr = "WDT (other watchdog)"; break;
    case ESP_RST_DEEPSLEEP: reasonStr = "DEEPSLEEP wake"; break;
    case ESP_RST_BROWNOUT:  reasonStr = "BROWNOUT (supply voltage sagged below threshold!)"; break;
    case ESP_RST_SDIO:      reasonStr = "SDIO"; break;
    default:                reasonStr = "UNKNOWN"; break;
  }

  Serial.print("[BOOT] Reset reason: ");
  Serial.println(reasonStr);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  printResetReason();

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

  Wire.begin();
  initSensors();

  // NOTE: DWIN intentionally NOT initialized here yet.
  // This build sends the initial device_connect payload, waits for
  // downlinks, handles RFID login/logout uplinks, and now also runs
  // the weld start/stop/data state machine driven by Cal_Current.
  initRadio();
  sendDeviceOnline();

  RFID_Init();

  initShutdownButton();

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
  // Wait for downlinks (TIME_SYNC / RFID_ACK) and watch for RFID card
  // taps (welderlogin/welderlogout uplinks), and for the shutdown button
  // being held (graceful power-down: closes out any open weld/login
  // before sending device_status offline and sleeping).
  checkAndProcessDownlink();
  handleRFID();
  checkShutdownButton();

  // Weld start/stop/data state machine -- only runs once an operator is
  // logged in. Cal_Current crossing WELD_CURRENT_THRESHOLD (config.h) is
  // what welder_start/welder_stop are keyed off; while welding is active,
  // welder_data goes out every WELD_DATA_INTERVAL ms (also config.h).
  if (loggedIn)
  {
    readSensors();

    if (!weldingStarted)
    {
      if (Cal_Current > WELD_CURRENT_THRESHOLD)
      {
        weldingStarted   = true;
        lastWeldDataTime = millis();

        sendWeldStart();

        Serial.println("[SYSTEM] WELDING STARTED");
      }
    }
    else
    {
      if (Cal_Current <= WELD_CURRENT_THRESHOLD)
      {
        weldingStarted = false;

        sendWeldStop();

        Serial.println("[SYSTEM] WELDING STOPPED");
      }
      else if (millis() - lastWeldDataTime >= WELD_DATA_INTERVAL)
      {
        lastWeldDataTime = millis();

        sendWeldData();
      }
    }
  }

  delay(100);
}
