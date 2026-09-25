// include/SX1262.h
//
// Arduino/ESP32 port of drivers/sx1262.py.
// Same command set, same register-level logic, same comments/rationale
// preserved from the MicroPython driver where they explain *why* (TCXO
// timing, image calibration bands, IQ inversion for downlinks, etc).
//
// Ported 1:1 method-for-method so behavior matches the MicroPython version:
//   reset, write_command, read_command, set_dio2_as_rf_switch,
//   set_regulator_mode, set_dio3_as_tcxo, calibrate_all, calibrate_image,
//   set_sync_word, get_status, init_lora, set_rf_frequency,
//   set_modulation_params, set_packet_params, set_pa_config, set_tx_params,
//   set_buffer_base_address, write_buffer, set_dio_irq_params,
//   get_irq_status, clear_irq_status, set_tx, set_rx,
//   get_rx_buffer_status, get_packet_status, read_buffer, transmit,
//   configure_rx, trigger_rx, wait_for_rx_result, trigger_rx_continuous,
//   wait_for_rx_continuous, receive.

#pragma once

#include <Arduino.h>
#include <SPI.h>

struct PacketStatus {
    float rssi_pkt_dbm;
    float snr_pkt_db;
    float signal_rssi_pkt_dbm;
};

class SX1262 {
public:
    // ---- Opcodes ----
    static constexpr uint8_t CMD_SET_STANDBY            = 0x80;
    static constexpr uint8_t CMD_SET_PACKET_TYPE         = 0x8A;
    static constexpr uint8_t CMD_SET_RF_FREQ             = 0x86;
    static constexpr uint8_t CMD_SET_PA_CONFIG           = 0x95;
    static constexpr uint8_t CMD_SET_TX_PARAMS           = 0x8E;
    static constexpr uint8_t CMD_WRITE_REGISTER          = 0x0D;
    static constexpr uint8_t CMD_READ_REGISTER           = 0x1D;
    static constexpr uint8_t CMD_SET_MODULATION_PARAMS   = 0x8B;
    static constexpr uint8_t CMD_SET_PACKET_PARAMS       = 0x8C;
    static constexpr uint8_t CMD_SET_BUFFER_BASE_ADDRESS = 0x8F;
    static constexpr uint8_t CMD_WRITE_BUFFER            = 0x0E;
    static constexpr uint8_t CMD_READ_BUFFER             = 0x1E;
    static constexpr uint8_t CMD_SET_TX                  = 0x83;
    static constexpr uint8_t CMD_SET_RX                  = 0x82;
    static constexpr uint8_t CMD_SET_DIO_IRQ_PARAMS      = 0x08;
    static constexpr uint8_t CMD_GET_IRQ_STATUS          = 0x12;
    static constexpr uint8_t CMD_CLEAR_IRQ_STATUS        = 0x02;
    static constexpr uint8_t CMD_SET_DIO3_AS_TCXO_CTRL   = 0x97;
    static constexpr uint8_t CMD_CALIBRATE               = 0x89;
    static constexpr uint8_t CMD_GET_STATUS              = 0xC0;
    static constexpr uint8_t CMD_GET_RX_BUFFER_STATUS    = 0x13;
    static constexpr uint8_t CMD_GET_PACKET_STATUS       = 0x14;
    static constexpr uint8_t CMD_SET_DIO2_AS_RF_SWITCH   = 0x9D;
    static constexpr uint8_t CMD_SET_REGULATOR_MODE      = 0x96;
    static constexpr uint8_t CMD_CALIBRATE_IMAGE         = 0x98;

    // LoRa Sync Word register address (SX126x datasheet 13.4.1)
    static constexpr uint16_t REG_LORA_SYNC_WORD_MSB = 0x0740;

    // IRQ bit masks (SX126x datasheet 13.3.1)
    static constexpr uint16_t IRQ_TX_DONE            = 0x0001;
    static constexpr uint16_t IRQ_RX_DONE            = 0x0002;
    static constexpr uint16_t IRQ_PREAMBLE_DETECTED  = 0x0004;
    static constexpr uint16_t IRQ_SYNC_WORD_VALID    = 0x0008;
    static constexpr uint16_t IRQ_HEADER_VALID       = 0x0010;
    static constexpr uint16_t IRQ_HEADER_ERR         = 0x0020;
    static constexpr uint16_t IRQ_CRC_ERR            = 0x0040;
    static constexpr uint16_t IRQ_TIMEOUT            = 0x0200;
    static constexpr uint16_t IRQ_ALL                = 0xFFFF;

    static constexpr uint32_t XTAL_FREQ = 32000000UL;
    static constexpr double FREQ_STEP = (double)XTAL_FREQ / (double)(1UL << 25); // ~0.9536743164 Hz/LSB

    SX1262(SPIClass &spi, uint8_t cs, uint8_t reset, uint8_t busy,
           uint8_t dio1, uint8_t rxen, uint8_t txen);

    void begin(uint32_t spiHz = 1000000);

    // ---- Low-level ----
    void reset();
    void waitBusy();
    void writeCommand(uint8_t opCode, const uint8_t *data = nullptr, size_t len = 0);
    void readCommand(uint8_t opcode, uint8_t *outBuf, size_t len);

    void setDio2AsRfSwitch(bool enable = true);
    void setRegulatorMode();
    void setDio3AsTcxo(float voltage = 1.8f, uint32_t timeoutMs = 10);
    void calibrateAll();
    void calibrateImage(const char *band = "863-870");
    void setSyncWord(bool pub = true);
    uint8_t getStatus();

    void initLora(float tcxoVoltage = 1.8f);

    // ---- RF configuration ----
    void setRfFrequency(uint32_t freqHz);
    void setModulationParams(uint8_t sf, uint32_t bwHz, uint8_t cr, int lowDataRateOptimize = -1);
    void setPacketParams(uint8_t payloadLen, uint16_t preambleLen = 8,
                          uint8_t headerType = 0x00, uint8_t crcType = 0x01, uint8_t invertIq = 0x00);
    void setPaConfig(uint8_t paDutyCycle = 0x04, uint8_t hpMax = 0x07,
                      uint8_t deviceSel = 0x00, uint8_t paLut = 0x01);
    void setTxParams(int8_t powerDbm, uint8_t rampTime = 0x04);

    // ---- Buffer / IRQ / TX/RX ----
    void setBufferBaseAddress(uint8_t txBase = 0x00, uint8_t rxBase = 0x00);
    void writeBuffer(const uint8_t *payload, size_t len, uint8_t offset = 0x00);
    void setDioIrqParams(uint16_t irqMask, uint16_t dio1Mask,
                          uint16_t dio2Mask = 0x0000, uint16_t dio3Mask = 0x0000);
    uint16_t getIrqStatus();
    void clearIrqStatus(uint16_t mask = IRQ_ALL);
    void setTx(uint32_t timeoutMs = 0);
    void setRx(uint32_t timeoutMs = 0);
    void getRxBufferStatus(uint8_t &payloadLen, uint8_t &rxStartPtr);
    PacketStatus getPacketStatus();
    void readBuffer(uint8_t offset, uint8_t length, uint8_t *outBuf);

    // ---- High-level orchestration ----
    bool transmit(const uint8_t *payload, size_t len, uint32_t freqHz,
                  uint8_t sf, uint32_t bwHz, uint8_t cr,
                  int8_t powerDbm, uint32_t timeoutMs = 4000);

    void configureRx(uint32_t freqHz, uint8_t sf, uint32_t bwHz, uint8_t cr,
                      uint8_t maxPayloadLen = 64);
    void triggerRx(uint32_t timeoutMs = 0);
    // Returns true if a packet was received; fills outBuf/outLen. false = nothing received.
    bool waitForRxResult(uint32_t timeoutMs, uint8_t *outBuf, uint8_t &outLen, uint32_t pollMs = 5);

    void triggerRxContinuous();
    // Blocks indefinitely until a valid packet arrives.
    void waitForRxContinuous(uint8_t *outBuf, uint8_t &outLen, uint32_t pollMs = 20);

    bool receive(uint32_t freqHz, uint8_t sf, uint32_t bwHz, uint8_t cr,
                 uint8_t *outBuf, uint8_t &outLen,
                 uint32_t timeoutMs = 3000, uint8_t maxPayloadLen = 64);

private:
    SPIClass &spi_;
    uint8_t cs_, reset_, busy_, dio1_, rxen_, txen_;
    SPISettings spiSettings_;

    static uint8_t voltageCode(float voltage);
    static uint8_t bwCode(uint32_t bwHz);
    static bool imageCalBand(const char *band, uint8_t &f1, uint8_t &f2);
};
