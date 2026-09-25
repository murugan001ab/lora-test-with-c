// include/config.h
//
// All pin assignments, RF parameters, and calibration/tuning constants for
// the ZWELDAQ firmware. No logic lives here — just #defines and consts,
// so any module can include this without pulling in other dependencies.

#pragma once
#include <Arduino.h>

// ================================================================
//                         DEVICE CONFIG
// ================================================================
// DEVICE_ID and ORGANIZATION_ID are set per-unit from platformio.ini's
// per-environment build_flags (-D DEVICE_ID=n -D ORGANIZATION_ID=m), so
// the same source tree builds a different identity for each physical
// device depending on which `pio run -e <env>` you flash with. The
// #ifndef guards below just give a sane default if config.h is ever
// compiled standalone (e.g. from an IDE that ignores build_flags).
#ifndef DEVICE_ID
#define DEVICE_ID        1
#endif

#ifndef ORGANIZATION_ID
#define ORGANIZATION_ID  1
#endif

// ================================================================
//                         SENSOR PINS
// ================================================================
#define Voltage_Sens     39
#define FeedRate_Sens    34
#define GasFlow_Sens     35

// ================================================================
//                         RFID PINS
// ================================================================
// RFID uses ESP32 VSPI peripheral through the global SPI object.
// SCK=14 MISO=12 MOSI=13 SS=15 RST=4
// ================================================================
#define RFID_SS_PIN      15
#define RFID_RST_PIN     4
#define RFID_SCK_PIN     14
#define RFID_MISO_PIN    12
#define RFID_MOSI_PIN    13

// ================================================================
//                         DWIN UART2
// ================================================================
#define DWIN_RX_PIN      16
#define DWIN_TX_PIN      17

// ================================================================
//                         LORA PINS (HSPI)
// ================================================================
// SCK=18 MISO=19 MOSI=23 CS=5 BUSY=25 DIO1=26 RST=27
// EBYTE RF switch: RXEN=32 TXEN=33
// ================================================================
#define LORA_SCK         18
#define LORA_MISO        19
#define LORA_MOSI        23
#define LORA_CS          5
#define LORA_RST         27
#define LORA_BUSY        25
#define LORA_DIO1        26
#define LORA_RXEN        32
#define LORA_TXEN        33

// ================================================================
//                       LORA RF PARAMETERS
// ================================================================
// LORA_FREQ is overridable per-device from platformio.ini's per-env
// build_flags (-D LORA_FREQ=xxxUL), same pattern as DEVICE_ID above.
// Needed because the Kerlink gateway has 8 channels -- each physical
// device can be pinned to a different channel frequency this way.
#ifndef LORA_FREQ
#define LORA_FREQ        865232500UL
#endif

#define LORA_SF          7
#define LORA_BW          125000UL
#define LORA_CR          1        // 4/5
#define TX_POWER         14       // dBm

// ================================================================
//                         MISC PINS
// ================================================================
#define LED_PIN          2

// Graceful shutdown button (ESP32 devkit BOOT button, active LOW)
#define SHUTDOWN_BTN_PIN 0

// ================================================================
//                         TIMING / THRESHOLDS
// ================================================================
const unsigned long SHUTDOWN_HOLD_MS   = 2000;
const unsigned long RFID_DEBOUNCE      = 800;
const unsigned long WELD_DATA_INTERVAL = 3000;
const float WELD_CURRENT_THRESHOLD     = 1.0f;  // TEMP: bench test only -- set back to 10.0f (or real min arc current) before field use

// ================================================================
//                       SENSOR CALIBRATION
// ================================================================
#define Ct_Cal_Factor    0.01855678
#define Vg_Cal_Factor    0.0245250376
#define FR_Cal_Factor    0.032334515
#define GF_Cal_Factor    0.0122189638

#define ref_volt         1.5
#define ref_volt1        3.30
