// src/SX1262.cpp
// Arduino/ESP32 port of drivers/sx1262.py -- see include/SX1262.h for notes.

#include "SX1262.h"

SX1262::SX1262(SPIClass &spi, uint8_t cs, uint8_t reset, uint8_t busy,
               uint8_t dio1, uint8_t rxen, uint8_t txen)
    : spi_(spi), cs_(cs), reset_(reset), busy_(busy), dio1_(dio1), rxen_(rxen), txen_(txen),
      spiSettings_(1000000, MSBFIRST, SPI_MODE0) {}

void SX1262::begin(uint32_t spiHz) {
    pinMode(cs_, OUTPUT);
    pinMode(reset_, OUTPUT);
    pinMode(busy_, INPUT);
    pinMode(dio1_, INPUT);
    pinMode(rxen_, OUTPUT);
    pinMode(txen_, OUTPUT);
    digitalWrite(cs_, HIGH);
    spiSettings_ = SPISettings(spiHz, MSBFIRST, SPI_MODE0);
}

// ------------------------------------------------------------------
// Low-level
// ------------------------------------------------------------------

void SX1262::reset() {
    Serial.println("[SX1262] Resetting transceiver...");
    digitalWrite(reset_, LOW);
    delay(10);
    digitalWrite(reset_, HIGH);
    delay(20);
    waitBusy();
    Serial.println("[SX1262] Reset complete.");
}

void SX1262::waitBusy() {
    uint32_t deadline = millis() + 1000;
    while (digitalRead(busy_) == HIGH) {
        if ((int32_t)(deadline - millis()) < 0) {
            Serial.println("[SX1262] ERROR: BUSY pin timeout!");
            // Match the Python driver's "raise" behavior by halting the op;
            // callers should treat prolonged BUSY as a hardware fault.
            return;
        }
        delay(1);
    }
}

void SX1262::writeCommand(uint8_t opCode, const uint8_t *data, size_t len) {
    waitBusy();
    spi_.beginTransaction(spiSettings_);
    digitalWrite(cs_, LOW);
    spi_.transfer(opCode);
    for (size_t i = 0; i < len; i++) spi_.transfer(data[i]);
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
}

void SX1262::readCommand(uint8_t opcode, uint8_t *outBuf, size_t len) {
    waitBusy();
    spi_.beginTransaction(spiSettings_);
    digitalWrite(cs_, LOW);
    spi_.transfer(opcode);
    spi_.transfer(0x00); // NOP while status is clocked out, mirrors read_command()
    for (size_t i = 0; i < len; i++) {
        outBuf[i] = spi_.transfer(0x00);
    }
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
}

void SX1262::setDio2AsRfSwitch(bool enable) {
    uint8_t d = enable ? 0x01 : 0x00;
    writeCommand(CMD_SET_DIO2_AS_RF_SWITCH, &d, 1);
}

void SX1262::setRegulatorMode() {
    // 0x01 = DC-DC (recommended if available)
    uint8_t d = 0x01;
    writeCommand(CMD_SET_REGULATOR_MODE, &d, 1);
}

uint8_t SX1262::voltageCode(float voltage) {
    // Mirrors TCXO_VOLTAGE dict in the Python driver.
    if (voltage == 1.6f) return 0x00;
    if (voltage == 1.7f) return 0x01;
    if (voltage == 1.8f) return 0x02;
    if (voltage == 2.2f) return 0x03;
    if (voltage == 2.4f) return 0x04;
    if (voltage == 2.7f) return 0x05;
    if (voltage == 3.0f) return 0x06;
    if (voltage == 3.3f) return 0x07;
    Serial.println("[SX1262] ERROR: Unsupported TCXO voltage, defaulting to 1.8V");
    return 0x02;
}

void SX1262::setDio3AsTcxo(float voltage, uint32_t timeoutMs) {
    // Ebyte E22-900M22S modules use an onboard TCXO -- without this the
    // PLL calibration/frequency reference can be wrong even though every
    // other command appears to succeed.
    // NOTE: verify the exact voltage against the E22-900M22S datasheet;
    // Ebyte has shipped different TCXO voltages across batches.
    uint8_t voltageCodeByte = voltageCode(voltage);
    uint32_t timeoutSteps = (uint32_t)((double)timeoutMs * 1000.0 / 15.625);
    uint8_t data[4] = {
        voltageCodeByte,
        (uint8_t)((timeoutSteps >> 16) & 0xFF),
        (uint8_t)((timeoutSteps >> 8) & 0xFF),
        (uint8_t)(timeoutSteps & 0xFF),
    };
    writeCommand(CMD_SET_DIO3_AS_TCXO_CTRL, data, 4);
    Serial.printf("[SX1262] DIO3 configured as TCXO control (%.1fV)\n", voltage);
}

void SX1262::calibrateAll() {
    // Run all calibration blocks (RC64k, RC13M, PLL, ADC, IMG). Must be
    // done after TCXO is configured / stabilized so the PLL calibrates
    // against the correct reference.
    uint8_t d = 0xFF;
    writeCommand(CMD_CALIBRATE, &d, 1);
    waitBusy();
    Serial.println("[SX1262] Calibration complete.");
}

bool SX1262::imageCalBand(const char *band, uint8_t &f1, uint8_t &f2) {
    // Semtech-documented ImageCalFreq byte pairs per SX1261/2 datasheet
    // section 9.2.1. CalibrateFunction alone only calibrates the image
    // using the chip's default 902-928 MHz (US915) range -- for any other
    // band (IN865/EU868 fall in 863-870 MHz) that generic calibration is
    // for the WRONG frequencies unless CalibrateImage is also called
    // explicitly with the matching band bytes.
    String b(band);
    if (b == "430-440") { f1 = 0x6B; f2 = 0x6F; return true; }
    if (b == "470-510") { f1 = 0x75; f2 = 0x81; return true; }
    if (b == "779-787") { f1 = 0xC1; f2 = 0xC5; return true; }
    if (b == "863-870") { f1 = 0xD7; f2 = 0xDB; return true; } // IN865 / EU868
    if (b == "902-928") { f1 = 0xE1; f2 = 0xE9; return true; } // US915
    return false;
}

void SX1262::calibrateImage(const char *band) {
    // Explicitly calibrate image rejection for the given frequency band.
    // Must be called after calibrateAll() -- CalibrateFunction's generic
    // pass does not cover this for any band other than the chip's
    // factory default (902-928 MHz).
    uint8_t f1, f2;
    if (!imageCalBand(band, f1, f2)) {
        Serial.printf("[SX1262] ERROR: Unknown calibration band: %s\n", band);
        return;
    }
    uint8_t data[2] = { f1, f2 };
    writeCommand(CMD_CALIBRATE_IMAGE, data, 2);
    waitBusy();
    Serial.printf("[SX1262] Image calibration complete for %s MHz band.\n", band);
}

void SX1262::setSyncWord(bool pub) {
    // LoRaWAN networks (ChirpStack, TTN, etc.) use the PUBLIC sync word
    // (0x3444). The SX1262 defaults to the PRIVATE sync word (0x1424) on
    // reset -- if this is never set, a LoRaWAN gateway will not recognize
    // your packets as valid LoRa preambles at all.
    uint16_t val = pub ? 0x3444 : 0x1424;
    uint8_t data[4] = {
        (uint8_t)((REG_LORA_SYNC_WORD_MSB >> 8) & 0xFF),
        (uint8_t)(REG_LORA_SYNC_WORD_MSB & 0xFF),
        (uint8_t)((val >> 8) & 0xFF),
        (uint8_t)(val & 0xFF),
    };
    writeCommand(CMD_WRITE_REGISTER, data, 4);
    Serial.printf("[SX1262] Sync word set to %s\n", pub ? "PUBLIC (0x3444)" : "PRIVATE (0x1424)");
}

uint8_t SX1262::getStatus() {
    // The status byte returns on MISO during the SECOND byte of the
    // command (opcode, then status while NOP is clocked out).
    waitBusy();
    spi_.beginTransaction(spiSettings_);
    digitalWrite(cs_, LOW);
    spi_.transfer(CMD_GET_STATUS);
    uint8_t status = spi_.transfer(0x00);
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
    return status;
}

void SX1262::initLora(float tcxoVoltage) {
    reset();

    // TCXO must be configured before calibration so the PLL locks
    // against the correct reference. Do this before SetStandby.
    setDio3AsTcxo(tcxoVoltage);
    calibrateAll();
    calibrateImage("863-870"); // IN865 falls in this band

    // Set Standby (RC)
    uint8_t d0 = 0x00;
    writeCommand(CMD_SET_STANDBY, &d0, 1);
    Serial.printf("[SX1262] Status after reset: 0x%02X\n", getStatus());

    // Set Packet Type to LoRa (0x01)
    uint8_t d1 = 0x01;
    writeCommand(CMD_SET_PACKET_TYPE, &d1, 1);
    setDio2AsRfSwitch(true);

    // LoRaWAN networks use the public sync word -- without this the
    // gateway won't recognize the packet preamble at all.
    setSyncWord(true);

    Serial.println("[SX1262] Initialized in LoRa mode.");
}

// ------------------------------------------------------------------
// RF configuration
// ------------------------------------------------------------------

void SX1262::setRfFrequency(uint32_t freqHz) {
    uint32_t freqReg = (uint32_t)((double)freqHz / FREQ_STEP);
    uint8_t data[4] = {
        (uint8_t)((freqReg >> 24) & 0xFF),
        (uint8_t)((freqReg >> 16) & 0xFF),
        (uint8_t)((freqReg >> 8) & 0xFF),
        (uint8_t)(freqReg & 0xFF),
    };
    writeCommand(CMD_SET_RF_FREQ, data, 4);
    Serial.printf("[SX1262] RF frequency set to %lu Hz\n", (unsigned long)freqHz);
}

uint8_t SX1262::bwCode(uint32_t bwHz) {
    switch (bwHz) {
        case 7810:   return 0x00;
        case 10420:  return 0x08;
        case 15630:  return 0x01;
        case 20830:  return 0x09;
        case 31250:  return 0x02;
        case 41670:  return 0x0A;
        case 62500:  return 0x03;
        case 125000: return 0x04;
        case 250000: return 0x05;
        case 500000: return 0x06;
        default:
            Serial.printf("[SX1262] ERROR: Unsupported bandwidth: %lu Hz\n", (unsigned long)bwHz);
            return 0x04; // fall back to 125kHz
    }
}

void SX1262::setModulationParams(uint8_t sf, uint32_t bwHz, uint8_t cr, int lowDataRateOptimize) {
    uint8_t bwVal = bwCode(bwHz);
    uint8_t ldro;
    if (lowDataRateOptimize < 0) {
        // LoRaWAN rule of thumb: enable LDRO when symbol time >= 16.38ms
        double symbolTimeMs = (double)(1UL << sf) / ((double)bwHz / 1000.0);
        ldro = (symbolTimeMs >= 16.38) ? 1 : 0;
    } else {
        ldro = (uint8_t)lowDataRateOptimize;
    }
    uint8_t data[4] = { sf, bwVal, cr, ldro };
    writeCommand(CMD_SET_MODULATION_PARAMS, data, 4);
    Serial.printf("[SX1262] Modulation set: SF%d, BW%luHz, CR4/%d, LDRO=%d\n",
                  sf, (unsigned long)bwHz, cr + 4, ldro);
}

void SX1262::setPacketParams(uint8_t payloadLen, uint16_t preambleLen,
                              uint8_t headerType, uint8_t crcType, uint8_t invertIq) {
    // header_type: 0x00 = explicit header (LoRaWAN uplinks), 0x01 = implicit
    // crc_type: 0x00 = off, 0x01 = on
    // invert_iq: 0x00 = standard (uplink), 0x01 = inverted (downlink)
    uint8_t data[6] = {
        (uint8_t)((preambleLen >> 8) & 0xFF),
        (uint8_t)(preambleLen & 0xFF),
        headerType,
        payloadLen,
        crcType,
        invertIq,
    };
    writeCommand(CMD_SET_PACKET_PARAMS, data, 6);
    Serial.printf("[SX1262] Packet params set: preamble=%d, payload_len=%d, header=%s, crc=%d, iq=%d\n",
                  preambleLen, payloadLen, headerType ? "implicit" : "explicit", crcType, invertIq);
}

void SX1262::setPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut) {
    // SX1262 PA config. Defaults are the datasheet-recommended values for
    // +22dBm max output on the SX1262 (device_sel=0x00).
    uint8_t data[4] = { paDutyCycle, hpMax, deviceSel, paLut };
    writeCommand(CMD_SET_PA_CONFIG, data, 4);
}

void SX1262::setTxParams(int8_t powerDbm, uint8_t rampTime) {
    // power_dbm: -17 to +22 dBm (signed byte, two's complement)
    uint8_t powerByte = (uint8_t)powerDbm;
    uint8_t data[2] = { powerByte, rampTime };
    writeCommand(CMD_SET_TX_PARAMS, data, 2);
    Serial.printf("[SX1262] TX power set to %d dBm\n", powerDbm);
}

// ------------------------------------------------------------------
// Buffer / IRQ / TX/RX
// ------------------------------------------------------------------

void SX1262::setBufferBaseAddress(uint8_t txBase, uint8_t rxBase) {
    uint8_t data[2] = { txBase, rxBase };
    writeCommand(CMD_SET_BUFFER_BASE_ADDRESS, data, 2);
}

void SX1262::writeBuffer(const uint8_t *payload, size_t len, uint8_t offset) {
    waitBusy();
    spi_.beginTransaction(spiSettings_);
    digitalWrite(cs_, LOW);
    spi_.transfer(CMD_WRITE_BUFFER);
    spi_.transfer(offset);
    for (size_t i = 0; i < len; i++) spi_.transfer(payload[i]);
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
    Serial.printf("[SX1262] Wrote %u bytes to TX buffer\n", (unsigned)len);
}

void SX1262::setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask) {
    uint8_t data[8] = {
        (uint8_t)((irqMask >> 8) & 0xFF), (uint8_t)(irqMask & 0xFF),
        (uint8_t)((dio1Mask >> 8) & 0xFF), (uint8_t)(dio1Mask & 0xFF),
        (uint8_t)((dio2Mask >> 8) & 0xFF), (uint8_t)(dio2Mask & 0xFF),
        (uint8_t)((dio3Mask >> 8) & 0xFF), (uint8_t)(dio3Mask & 0xFF),
    };
    writeCommand(CMD_SET_DIO_IRQ_PARAMS, data, 8);
}

uint16_t SX1262::getIrqStatus() {
    uint8_t raw[2];
    readCommand(CMD_GET_IRQ_STATUS, raw, 2);
    return ((uint16_t)raw[0] << 8) | raw[1];
}

void SX1262::clearIrqStatus(uint16_t mask) {
    uint8_t data[2] = { (uint8_t)((mask >> 8) & 0xFF), (uint8_t)(mask & 0xFF) };
    writeCommand(CMD_CLEAR_IRQ_STATUS, data, 2);
}

void SX1262::setTx(uint32_t timeoutMs) {
    // Timeout unit on the radio is 15.625us steps. timeoutMs=0 means no
    // timeout (TX runs until TxDone).
    uint32_t timeoutSteps = timeoutMs ? (uint32_t)((double)timeoutMs * 1000.0 / 15.625) : 0;
    uint8_t data[3] = {
        (uint8_t)((timeoutSteps >> 16) & 0xFF),
        (uint8_t)((timeoutSteps >> 8) & 0xFF),
        (uint8_t)(timeoutSteps & 0xFF),
    };
    writeCommand(CMD_SET_TX, data, 3);
}

void SX1262::setRx(uint32_t timeoutMs) {
    // timeoutMs=0 means a single receive with no radio timeout (waits
    // indefinitely for RxDone) -- callers should still enforce their own
    // wall-clock deadline while polling DIO1.
    uint32_t timeoutSteps = timeoutMs ? (uint32_t)((double)timeoutMs * 1000.0 / 15.625) : 0;
    uint8_t data[3] = {
        (uint8_t)((timeoutSteps >> 16) & 0xFF),
        (uint8_t)((timeoutSteps >> 8) & 0xFF),
        (uint8_t)(timeoutSteps & 0xFF),
    };
    writeCommand(CMD_SET_RX, data, 3);
}

void SX1262::getRxBufferStatus(uint8_t &payloadLen, uint8_t &rxStartPtr) {
    uint8_t raw[2];
    readCommand(CMD_GET_RX_BUFFER_STATUS, raw, 2);
    payloadLen = raw[0];
    rxStartPtr = raw[1];
}

PacketStatus SX1262::getPacketStatus() {
    // Returns (rssi_pkt_dbm, snr_pkt_db, signal_rssi_pkt_dbm) for the most
    // recently received LoRa packet (SX126x datasheet 13.5.3). Must be
    // called before the next SetRx/SetTx, since it reflects only the
    // last demodulated packet.
    uint8_t raw[3];
    readCommand(CMD_GET_PACKET_STATUS, raw, 3);
    PacketStatus ps;
    ps.rssi_pkt_dbm = -raw[0] / 2.0f;
    int8_t snrRaw = (int8_t)raw[1]; // two's complement, matches Python's >127 wraparound
    ps.snr_pkt_db = snrRaw / 4.0f;
    ps.signal_rssi_pkt_dbm = -raw[2] / 2.0f;
    return ps;
}

void SX1262::readBuffer(uint8_t offset, uint8_t length, uint8_t *outBuf) {
    waitBusy();
    spi_.beginTransaction(spiSettings_);
    digitalWrite(cs_, LOW);
    spi_.transfer(CMD_READ_BUFFER);
    spi_.transfer(offset);
    spi_.transfer(0x00); // NOP byte, matches Python's 3-byte header before data
    for (uint8_t i = 0; i < length; i++) {
        outBuf[i] = spi_.transfer(0x00);
    }
    digitalWrite(cs_, HIGH);
    spi_.endTransaction();
}

// ------------------------------------------------------------------
// High-level orchestration
// ------------------------------------------------------------------

bool SX1262::transmit(const uint8_t *payload, size_t len, uint32_t freqHz,
                       uint8_t sf, uint32_t bwHz, uint8_t cr,
                       int8_t powerDbm, uint32_t timeoutMs) {
    // Full LoRa TX chain:
    // Standby -> Packet Type -> RF Freq -> Modulation Params -> Packet Params
    // -> PA Config -> TX Params -> Buffer Base Addr -> Write Payload -> DIO IRQ -> Set TX
    // Blocks (polling DIO1/IRQ) until TxDone or timeout. Returns true on success.

    // RF switch -> TX
    digitalWrite(rxen_, LOW);
    digitalWrite(txen_, HIGH);
    delay(1);
    Serial.println("RF SWITCH -> TX");

    setRegulatorMode();
    uint8_t standby = 0x01;
    writeCommand(CMD_SET_STANDBY, &standby, 1);
    uint8_t packetType = 0x01;
    writeCommand(CMD_SET_PACKET_TYPE, &packetType, 1);
    setSyncWord(true);

    setRfFrequency(freqHz);
    setModulationParams(sf, bwHz, cr);
    setPacketParams((uint8_t)len);
    setPaConfig();
    setTxParams(powerDbm);

    setBufferBaseAddress(0x00, 0x00);
    writeBuffer(payload, len);

    // Route TxDone + Timeout IRQs to DIO1 so we can poll it
    setDioIrqParams(IRQ_TX_DONE | IRQ_TIMEOUT, IRQ_TX_DONE | IRQ_TIMEOUT);
    clearIrqStatus();

    Serial.println("[SX1262] Starting TX...");
    setTx(timeoutMs);

    uint32_t deadline = millis() + timeoutMs + 500;

    while (true) {
        uint16_t irq = getIrqStatus();
        if (irq != 0) break;

        if ((int32_t)(deadline - millis()) < 0) {
            Serial.println("[SX1262] ERROR: TX timed out waiting on DIO1");
            clearIrqStatus();
            return false;
        }
        delay(20);
    }

    uint16_t status = getIrqStatus();
    clearIrqStatus();

    if (status & IRQ_TX_DONE) {
        Serial.println("[SX1262] TX complete (TxDone).");
        return true;
    }
    if (status & IRQ_TIMEOUT) {
        Serial.println("[SX1262] ERROR: TX failed: radio-reported timeout.");
        return false;
    }

    Serial.printf("[SX1262] ERROR: TX finished with unexpected IRQ status: 0x%04X\n", status);
    return false;
}

void SX1262::configureRx(uint32_t freqHz, uint8_t sf, uint32_t bwHz, uint8_t cr, uint8_t maxPayloadLen) {
    // Pre-configure the radio for reception WITHOUT opening the RX window.
    // Call this as early as possible (e.g. right after transmit() returns)
    // so that all the SPI/register setup latency (~10 commands, each with
    // a busy-wait) is absorbed *before* the precise RX1/RX2 delay
    // countdown, rather than after it. Follow with triggerRx() timed to
    // land as close as possible to the exact target moment.
    //
    // Downlinks are IQ-inverted relative to uplinks -- invert_iq=1 here
    // matches what the gateway actually transmits; leaving this at 0
    // (the uplink default) means the radio will fail to demodulate a
    // Join-Accept even if timing and frequency are correct.

    // RF switch -> RX
    digitalWrite(txen_, LOW);
    digitalWrite(rxen_, HIGH);
    Serial.println("RF SWITCH -> RX");
    delay(1);

    setRegulatorMode();
    // STDBY_XOSC (0x01), matching transmit() -- STDBY_RC would force an
    // extra TCXO warm-up delay once SetRx is issued, silently shifting
    // the actual start of listening later than the RX1/RX2 timing
    // calculated in main.py assumes.
    uint8_t standby = 0x01;
    writeCommand(CMD_SET_STANDBY, &standby, 1);
    uint8_t packetType = 0x01;
    writeCommand(CMD_SET_PACKET_TYPE, &packetType, 1);
    setSyncWord(true);

    setRfFrequency(freqHz);
    setModulationParams(sf, bwHz, cr);
    setBufferBaseAddress(0x00, 0x00);

    setDioIrqParams(
        IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CRC_ERR | IRQ_HEADER_ERR | IRQ_PREAMBLE_DETECTED,
        // PREAMBLE_DETECTED is intentionally left OUT of dio1_mask: it fires
        // the instant a preamble is merely seen, long before the header/sync/
        // payload are actually demodulated. Routing it to DIO1 caused
        // waitForRxResult() to treat 'preamble seen' as 'operation done'
        // and bail out mid-reception. DIO1 should only fire on true
        // completion events.
        IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CRC_ERR | IRQ_HEADER_ERR
    );
    clearIrqStatus();

    setPacketParams(maxPayloadLen, 8, 0x00, 0x01, 0x00);
    Serial.println("[SX1262] RX pre-configured, ready to trigger.");
}

void SX1262::triggerRx(uint32_t timeoutMs) {
    // Open the RX window right now. configureRx() must already have been
    // called -- this is just the single SetRx SPI command, kept as
    // lightweight as possible so it can be timed precisely.
    Serial.printf("[SX1262] Opening RX window (%lums)...\n", (unsigned long)timeoutMs);
    setRx(timeoutMs);
}

bool SX1262::waitForRxResult(uint32_t timeoutMs, uint8_t *outBuf, uint8_t &outLen, uint32_t pollMs) {
    // Poll for an RX result after triggerRx() has opened the window.
    // Only these bits mean the RX operation is actually OVER. Excluding
    // PREAMBLE_DETECTED here is the critical part -- see configureRx().
    const uint16_t COMPLETION_MASK = IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CRC_ERR | IRQ_HEADER_ERR;
    uint32_t deadline = millis() + timeoutMs + 500;

    while (true) {
        uint16_t irq = getIrqStatus();
        if (irq & COMPLETION_MASK) break;

        if ((int32_t)(deadline - millis()) < 0) {
            Serial.println("[SX1262] RX window closed: no DIO1 activity (nothing received).");
            clearIrqStatus();
            return false;
        }
        delay(pollMs);
    }

    uint16_t status = getIrqStatus();
    clearIrqStatus();

    if (status & (IRQ_CRC_ERR | IRQ_HEADER_ERR)) {
        Serial.printf("[SX1262] RX failed: CRC/header error (IRQ=0x%04X).\n", status);
        return false;
    }
    if (!(status & IRQ_RX_DONE)) {
        if (status & IRQ_PREAMBLE_DETECTED) {
            // Radio saw RF energy shaped like a LoRa preamble but never
            // locked sync/header -- points at a parameter mismatch
            // (freq/SF/BW/IQ) rather than "nothing arrived".
            Serial.printf("[SX1262] RX timed out, but a preamble WAS detected (IRQ=0x%04X) "
                          "-- signal is arriving, check SF/BW/IQ/sync-word match.\n", status);
        } else {
            // No PreambleDetected bit at all -- the chip never saw
            // anything resembling RF energy during the window.
            Serial.printf("[SX1262] RX window closed without RxDone (IRQ=0x%04X), "
                          "no preamble ever detected -- likely nothing reached the antenna.\n", status);
        }
        return false;
    }

    uint8_t payloadLen, startPtr;
    getRxBufferStatus(payloadLen, startPtr);
    readBuffer(startPtr, payloadLen, outBuf);
    outLen = payloadLen;
    Serial.printf("[SX1262] Received %d bytes.\n", payloadLen);
    return true;
}

void SX1262::triggerRxContinuous() {
    // Open a CONTINUOUS RX window right now (SetRx timeout=0xFFFFFF, per
    // SX126x datasheet 13.1.7). configureRx() must already have been
    // called. Unlike a single-shot window, the radio re-arms itself after
    // every demodulated packet (good or bad CRC) with no host action
    // required -- this is what lets the caller "keep listening
    // continuously" without ever reissuing SetRx.
    Serial.println("[SX1262] Opening continuous RX window (listening until a downlink arrives)...");
    uint8_t data[3] = { 0xFF, 0xFF, 0xFF };
    writeCommand(CMD_SET_RX, data, 3);
}

void SX1262::waitForRxContinuous(uint8_t *outBuf, uint8_t &outLen, uint32_t pollMs) {
    // Blocks indefinitely (no deadline) in continuous RX mode until a
    // valid packet (RxDone) is demodulated, returning its payload bytes.
    // CRC/header errors are logged and discarded without ever touching
    // SetRx again -- the hardware keeps listening on its own, so this
    // never "restarts the receive window".
    while (true) {
        uint16_t irq = getIrqStatus();

        if (irq & IRQ_RX_DONE) {
            clearIrqStatus();
            uint8_t payloadLen, startPtr;
            getRxBufferStatus(payloadLen, startPtr);
            readBuffer(startPtr, payloadLen, outBuf);
            outLen = payloadLen;
            Serial.printf("[SX1262] Received %d bytes (continuous RX).\n", payloadLen);
            return;
        }

        if (irq & (IRQ_CRC_ERR | IRQ_HEADER_ERR)) {
            Serial.printf("[SX1262] Discarding corrupt frame (IRQ=0x%04X), still listening...\n", irq);
            clearIrqStatus();
            continue;
        }

        delay(pollMs);
    }
}

bool SX1262::receive(uint32_t freqHz, uint8_t sf, uint32_t bwHz, uint8_t cr,
                      uint8_t *outBuf, uint8_t &outLen,
                      uint32_t timeoutMs, uint8_t maxPayloadLen) {
    // Convenience wrapper: configure + trigger + wait in one call. Fine
    // for RX2 (wide window, timing less critical) or standalone testing.
    // For RX1, where sub-100ms precision matters, call configureRx()
    // early and triggerRx()/waitForRxResult() separately instead.
    configureRx(freqHz, sf, bwHz, cr, maxPayloadLen);
    triggerRx(timeoutMs);
    return waitForRxResult(timeoutMs, outBuf, outLen);
}
