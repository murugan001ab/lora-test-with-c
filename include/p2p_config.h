// include/p2p_config.h
//
// Port of p2p/p2p_config.py.
// Config for simple raw LoRa point-to-point test (no LoRaWAN, no OTAA).
// Same pinout as the MicroPython config (VSPI + Ebyte E22 RF-switch pins).
// Flash the SAME firmware to both boards -- only difference is which
// build you flash: the TX role (this file's main.cpp) vs an RX-only
// counterpart if you make one later.

#pragma once
#include <stdint.h>

namespace P2P {
    constexpr uint8_t PIN_SCK   = 18;
    constexpr uint8_t PIN_MOSI  = 23;
    constexpr uint8_t PIN_MISO  = 19;
    constexpr uint8_t PIN_CS    = 5;
    constexpr uint8_t PIN_RESET = 27;
    constexpr uint8_t PIN_BUSY  = 25;
    constexpr uint8_t PIN_DIO1  = 26;
    constexpr uint8_t PIN_LED   = 2;
    constexpr uint8_t PIN_TXEN  = 33;
    constexpr uint8_t PIN_RXEN  = 32;

    // Any legal in-band frequency works for P2P -- doesn't have to be a
    // ChirpStack channel since there's no gateway involved. Reusing one of
    // the IN865 channels here just because it's already known-good on
    // this E22-900M22S (IN865/EU868 band) hardware.
    constexpr uint32_t FREQ_HZ = 865232500UL;

    constexpr uint8_t SF = 7;
    constexpr uint32_t BW_HZ = 125000;
    constexpr uint8_t CR = 1;        // 1 = 4/5
    constexpr int8_t TX_POWER = 14;  // dBm

    // NOTE: SX1262::transmit()/configureRx() call setSyncWord(true)
    // unconditionally, so both ends use the public (0x3444) sync word no
    // matter what. That's harmless for P2P (no gateway involved -- both
    // ends just need to match, and they do) so this is left as-is rather
    // than touching the shared driver.
}
