// include/radio_sx1262.h
//
// SX1262 LoRa driver for the ZWELDAQ raw uplink/downlink protocol (not
// LoRaWAN — a fixed frequency/SF/BW with a custom downlink command byte).
// This is a self-contained register-level implementation kept separate
// from the generic SX1262 class in this repo (used by legacy/p2p_main.cpp)
// because that class's initLora() adds TCXO configuration and image
// calibration this hardware's original firmware never performed — reusing
// it here would be a behavior change on radio hardware, not just a
// refactor. Only these three functions are exposed; everything else
// (SPI framing, IRQ handling, packet params) is internal to the .cpp.

#pragma once
#include <Arduino.h>

// Configures RF-switch/CS/BUSY/DIO1/RST pins, starts the HSPI bus, resets
// and configures the radio (freq/SF/BW/CR/power from config.h), and
// leaves it in continuous RX. Call once from setup().
void initRadio();

// Sends one packet: switches the RF switch to TX, transmits, waits up to
// 5s for TxDone/timeout, then switches back to continuous RX. Returns
// true on TxDone, false on failure/timeout.
bool sendLoRa(const String &payload);

// Non-blocking — call every loop() iteration. If a downlink packet has
// arrived, reads it, dispatches it (currently: 0x01 TIME_SYNC, 4-byte
// big-endian Unix epoch seconds), and leaves the radio listening.
void checkAndProcessDownlink();
