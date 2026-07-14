// src/main.cpp
//
// Port of p2p/p2p_tx.py.
// Simple raw LoRa sender. No LoRaWAN, no OTAA, no gateway -- just
// SX1262 -> SX1262 point-to-point. Sends an incrementing counter packet
// every few seconds, then listens (continuous RX) for a downlink before
// repeating -- exactly like the MicroPython original's loop structure
// (once a downlink has been seen, it stops re-arming continuous RX,
// mirroring the `if not downlink:` guard from p2p_tx.py).
//
// Flash this same build to both boards to test; run a receive-only
// counterpart if you want a dedicated RX board instead.

#include <Arduino.h>
#include <SPI.h>
#include "SX1262.h"
#include "p2p_config.h"

static SPIClass radioSpi(VSPI);
static SX1262 radio(radioSpi, P2P::PIN_CS, P2P::PIN_RESET, P2P::PIN_BUSY,
                     P2P::PIN_DIO1, P2P::PIN_RXEN, P2P::PIN_TXEN);

static uint32_t counter = 0;
static bool haveDownlink = false; // mirrors Python's `downlink = ""` falsy check

void setup() {
    Serial.begin(115200);
    delay(1000);

    pinMode(P2P::PIN_LED, OUTPUT);
    digitalWrite(P2P::PIN_LED, HIGH); // off (active-low on most of these boards)

    radioSpi.begin(P2P::PIN_SCK, P2P::PIN_MISO, P2P::PIN_MOSI, P2P::PIN_CS);
    radio.begin(1000000);

    radio.initLora();
}

void loop() {
    counter++;
    char payload[32];
    int len = snprintf(payload, sizeof(payload), "hello #%lu", (unsigned long)counter);

    Serial.printf("[P2P_TX] Sending: %s\n", payload);

    digitalWrite(P2P::PIN_LED, LOW);

    bool ok = radio.transmit(
        (const uint8_t *)payload, len,
        P2P::FREQ_HZ, P2P::SF, P2P::BW_HZ, P2P::CR, P2P::TX_POWER
    );

    digitalWrite(P2P::PIN_LED, HIGH);

    if (!ok) {
        Serial.println("[P2P_TX] TX failed.");
        return; // matches Python's `continue`
    }

    Serial.println("[P2P_TX] TX done.");
    Serial.println("[P2P_TX] Waiting for downlink...");

    if (!haveDownlink) {
        radio.configureRx(P2P::FREQ_HZ, P2P::SF, P2P::BW_HZ, P2P::CR, 64);
        radio.triggerRxContinuous();

        while (true) {
            uint8_t buf[64];
            uint8_t rxLen = 0;
            radio.waitForRxContinuous(buf, rxLen); // blocks until a packet arrives

            haveDownlink = true;

            PacketStatus ps = radio.getPacketStatus();

            Serial.printf("[P2P_TX] Downlink (%d bytes): ", rxLen);
            for (uint8_t i = 0; i < rxLen; i++) Serial.write(buf[i]);
            Serial.println();

            Serial.printf("[P2P_TX] RSSI=%.1f SNR=%.1f\n", ps.rssi_pkt_dbm, ps.snr_pkt_db);
            break;
        }
    }

    delay(10000);
}
