#include "radio_sx1262.h"
#include "config.h"
#include "state.h"
#include "time_utils.h"

#include <SPI.h>

// ================================================================
//                       LORA COMMANDS
// ================================================================
static const uint8_t CMD_STANDBY              = 0x80;
static const uint8_t CMD_SET_PACKET_TYPE      = 0x8A;
static const uint8_t CMD_SET_RF_FREQ          = 0x86;
static const uint8_t CMD_SET_MODULATION       = 0x8B;
static const uint8_t CMD_SET_PACKET           = 0x8C;
static const uint8_t CMD_SET_BUFFER           = 0x8F;
static const uint8_t CMD_WRITE_BUFFER         = 0x0E;
static const uint8_t CMD_READ_BUFFER          = 0x1E;
static const uint8_t CMD_GET_RX_BUFFER_STATUS = 0x13;
static const uint8_t CMD_SET_TX               = 0x83;
static const uint8_t CMD_SET_RX               = 0x82;
static const uint8_t CMD_GET_IRQ              = 0x12;
static const uint8_t CMD_CLEAR_IRQ            = 0x02;
static const uint8_t CMD_SET_IRQ              = 0x08;
static const uint8_t CMD_SET_TX_PARAMS        = 0x8E;
static const uint8_t CMD_SET_PA_CONFIG        = 0x95;
static const uint8_t CMD_DIO2_RF_SWITCH       = 0x9D;
static const uint8_t CMD_GET_STATUS           = 0xC0;
static const uint8_t CMD_SET_DIO3_TCXO_CTRL   = 0x97;
static const uint8_t CMD_CALIBRATE            = 0x89;
static const uint8_t CMD_CALIBRATE_IMAGE      = 0x98;
static const uint8_t CMD_WRITE_REGISTER       = 0x0D;
static const uint8_t CMD_SET_REGULATOR_MODE   = 0x96;
static const uint16_t REG_LORA_SYNC_WORD_MSB  = 0x0740;

static const uint16_t IRQ_TX_DONE   = 0x0001;
static const uint16_t IRQ_RX_DONE   = 0x0002;
static const uint16_t IRQ_TIMEOUT   = 0x0200;
static const uint16_t IRQ_CRC_ERROR = 0x0040;

static const double FREQ_STEP = 32000000.0 / 33554432.0;

static SPIClass radioSPI(HSPI);
static SPISettings radioSPISettings(1000000, MSBFIRST, SPI_MODE0);

// ================================================================
//                       LOW-LEVEL SPI
// ================================================================

static void waitBusy()
{
  unsigned long start = millis();

  while (digitalRead(LORA_BUSY) == HIGH)
  {
    if (millis() - start > 1000)
    {
      Serial.println("[SX1262] ERROR: BUSY pin timeout!");
      return;
    }
    delay(1);
  }
}

static void loraSelect()   { digitalWrite(LORA_CS, LOW); }
static void loraDeselect() { digitalWrite(LORA_CS, HIGH); }

static void writeCommand(uint8_t command, const uint8_t *data, uint8_t length)
{
  waitBusy();
  radioSPI.beginTransaction(radioSPISettings);
  loraSelect();

  radioSPI.transfer(command);
  for (uint8_t i = 0; i < length; i++)
  {
    radioSPI.transfer(data[i]);
  }

  loraDeselect();
  radioSPI.endTransaction();
  waitBusy();
}

static void readCommand(uint8_t command, uint8_t *data, uint8_t length)
{
  waitBusy();
  radioSPI.beginTransaction(radioSPISettings);
  loraSelect();

  radioSPI.transfer(command);
  radioSPI.transfer(0x00);

  for (uint8_t i = 0; i < length; i++)
  {
    data[i] = radioSPI.transfer(0x00);
  }

  loraDeselect();
  radioSPI.endTransaction();
}

static void getRxBufferStatus(uint8_t &payloadLength, uint8_t &startPointer)
{
  uint8_t data[2];
  readCommand(CMD_GET_RX_BUFFER_STATUS, data, 2);

  payloadLength = data[0];
  startPointer  = data[1];
}

static void readBuffer(uint8_t offset, uint8_t *buffer, uint8_t length)
{
  waitBusy();
  radioSPI.beginTransaction(radioSPISettings);
  loraSelect();

  radioSPI.transfer(CMD_READ_BUFFER);
  radioSPI.transfer(offset);
  radioSPI.transfer(0x00);

  for (uint8_t i = 0; i < length; i++)
  {
    buffer[i] = radioSPI.transfer(0x00);
  }

  loraDeselect();
  radioSPI.endTransaction();
}

// ================================================================
//                       RADIO CONFIGURATION
// ================================================================

static void setStandby()
{
  uint8_t data = 0x01;
  writeCommand(CMD_STANDBY, &data, 1);
}

static void radioReset()
{
  Serial.println("[SX1262] Resetting...");

  digitalWrite(LORA_RST, LOW);
  delay(20);
  digitalWrite(LORA_RST, HIGH);
  delay(100);

  waitBusy();

  Serial.println("[SX1262] Reset complete");
}

static void setDio3AsTcxoCtrl()
{
  // Ebyte SX1262 modules run off a TCXO powered via the chip's DIO3
  // pin, enabled purely by this SPI command (no ESP32 GPIO involved).
  // Without this, the oscillator may never start reliably -> every RF
  // command still "works" over SPI but TX/RX never actually completes.
  //
  // Voltage byte per SX126x datasheet Table 13-38:
  //   0x00=1.6V 0x01=1.7V 0x02=1.8V 0x03=2.2V
  //   0x04=2.4V 0x05=2.7V 0x06=3.0V 0x07=3.3V
  // Confirmed 1.8V against the same Ebyte E22-900M22S module in the
  // known-working lora-test-with-c reference driver (which reaches
  // the Kerlink gateway successfully) -- do not change without
  // re-verifying against that reference or the module's datasheet.
  uint8_t data[4];
  data[0] = 0x02;               // TCXO supply voltage = 1.8V
  uint32_t delay = 320;        // startup delay, units of 15.625 us (~5 ms)
  data[1] = (delay >> 16) & 0xFF;
  data[2] = (delay >> 8) & 0xFF;
  data[3] = delay & 0xFF;

  writeCommand(CMD_SET_DIO3_TCXO_CTRL, data, 4);
}

static void calibrate()
{
  uint8_t data = 0x7F; // calibrate all blocks (RC64k, RC13M, PLL, ADC, IMG)
  writeCommand(CMD_CALIBRATE, &data, 1);
  delay(10); // datasheet: allow calibration to finish before next command
}

static void calibrateImage863to870()
{
  // CalibrateFunction's generic pass above only calibrates image
  // rejection for the chip's factory-default band (902-928 MHz /
  // US915). Our actual TX band is 863-870 MHz (IN865/EU868), so we
  // must explicitly calibrate image rejection for THIS band too, or
  // RX sensitivity in that band is degraded. Byte pair per SX1261/2
  // datasheet section 9.2.1.
  uint8_t data[2] = { 0xD7, 0xDB };
  writeCommand(CMD_CALIBRATE_IMAGE, data, 2);
  waitBusy();
}

static void setRegulatorModeDCDC()
{
  uint8_t data = 0x01; // DC-DC (recommended over LDO when available)
  writeCommand(CMD_SET_REGULATOR_MODE, &data, 1);
}

static void setSyncWordPublic()
{
  // The SX1262 defaults to the PRIVATE LoRa sync word (0x1424) on
  // reset. A LoRaWAN-style gateway packet forwarder (Kerlink/lorad,
  // ChirpStack, TTN, etc.) only recognizes the PUBLIC sync word
  // (0x3444) as a valid LoRa preamble -- with the wrong sync word the
  // gateway silently never sees the packet at the PHY layer at all,
  // even though the ESP32 itself reports TX DONE successfully. This
  // was confirmed working against the same Kerlink gateway in the
  // lora-test-with-c reference driver.
  uint8_t data[4] = {
    (uint8_t)((REG_LORA_SYNC_WORD_MSB >> 8) & 0xFF),
    (uint8_t)(REG_LORA_SYNC_WORD_MSB & 0xFF),
    0x34, 0x44 // public sync word
  };
  writeCommand(CMD_WRITE_REGISTER, data, 4);
}

static void setPacketTypeLoRa()
{
  uint8_t data[1] = { 0x01 };
  writeCommand(CMD_SET_PACKET_TYPE, data, 1);
}

static void setFrequency(uint32_t frequency)
{
  uint32_t frf = (uint32_t)((double)frequency / FREQ_STEP);

  uint8_t data[4];
  data[0] = (frf >> 24) & 0xFF;
  data[1] = (frf >> 16) & 0xFF;
  data[2] = (frf >> 8) & 0xFF;
  data[3] = frf & 0xFF;

  writeCommand(CMD_SET_RF_FREQ, data, 4);
}

static void setModulation()
{
  uint8_t data[4];
  data[0] = LORA_SF;
  data[1] = 0x04; // BW 125 kHz
  data[2] = LORA_CR;
  data[3] = 0x00;

  writeCommand(CMD_SET_MODULATION, data, 4);
}

static void setPacketParameters(uint8_t payloadLength)
{
  // payloadLength = the actual number of bytes the radio will transmit
  // (TX) from the buffer, or the max buffer size to allocate (RX --
  // the real received length comes from GetRxBufferStatus afterward,
  // so 0xFF/255 is safe there). This MUST be set to the real message
  // length before every TX call in explicit-header mode, or the SX1262
  // transmits stale/garbage buffer bytes past the real payload,
  // producing a corrupted, oversized packet that the gateway's CRC
  // check silently drops -- even though TxDone still fires locally.
  uint8_t data[6];
  data[0] = 0x00; // preamble = 8
  data[1] = 0x08;
  data[2] = 0x00; // explicit header
  data[3] = payloadLength;
  data[4] = 0x01; // CRC ON
  data[5] = 0x00; // normal IQ

  writeCommand(CMD_SET_PACKET, data, 6);
}

static void setBufferBase()
{
  uint8_t data[2] = { 0x00, 0x00 };
  writeCommand(CMD_SET_BUFFER, data, 2);
}

static void setTxParams()
{
  uint8_t data[2];
  data[0] = TX_POWER;
  data[1] = 0x04;

  writeCommand(CMD_SET_TX_PARAMS, data, 2);
}

static void setPAConfig()
{
  uint8_t data[4] = { 0x04, 0x07, 0x00, 0x01 };
  writeCommand(CMD_SET_PA_CONFIG, data, 4);
}

static void enableDio2RF()
{
  uint8_t data = 0x01;
  writeCommand(CMD_DIO2_RF_SWITCH, &data, 1);
}

static void setIRQ()
{
  uint8_t data[8];
  uint16_t irqMask = IRQ_TX_DONE | IRQ_RX_DONE | IRQ_TIMEOUT | IRQ_CRC_ERROR;

  data[0] = (irqMask >> 8) & 0xFF;
  data[1] = irqMask & 0xFF;
  data[2] = data[0];
  data[3] = data[1];
  data[4] = 0x00;
  data[5] = 0x00;
  data[6] = 0x00;
  data[7] = 0x00;

  writeCommand(CMD_SET_IRQ, data, 8);
}

static void clearIRQ()
{
  uint8_t data[2] = { 0xFF, 0xFF };
  writeCommand(CMD_CLEAR_IRQ, data, 2);
}

static uint16_t getIRQ()
{
  uint8_t data[2];
  readCommand(CMD_GET_IRQ, data, 2);
  return ((uint16_t)data[0] << 8) | data[1];
}

static uint8_t getStatus()
{
  // GetStatus (0xC0) returns one status byte on the FIRST byte
  // clocked back (the NOP response), bits [6:4] = chip mode:
  //   2 = STBY_RC   3 = STBY_XOSC   4 = FS   5 = RX   6 = TX
  waitBusy();
  radioSPI.beginTransaction(radioSPISettings);
  loraSelect();
  uint8_t status = radioSPI.transfer(CMD_GET_STATUS);
  radioSPI.transfer(0x00);
  loraDeselect();
  radioSPI.endTransaction();
  return (status >> 4) & 0x07;
}

static void writePayload(const String &payload)
{
  waitBusy();
  radioSPI.beginTransaction(radioSPISettings);
  loraSelect();

  radioSPI.transfer(CMD_WRITE_BUFFER);
  radioSPI.transfer(0x00);

  for (size_t i = 0; i < payload.length(); i++)
  {
    radioSPI.transfer((uint8_t)payload[i]);
  }

  loraDeselect();
  radioSPI.endTransaction();
  waitBusy();
}

static void startTX()
{
  uint8_t data[3] = { 0x00, 0x00, 0x00 };
  writeCommand(CMD_SET_TX, data, 3);
}

static void startRX()
{
  uint8_t data[3] = { 0xFF, 0xFF, 0xFF };
  writeCommand(CMD_SET_RX, data, 3);
}

static void resumeListening()
{
  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, HIGH);
  delay(2);

  setPacketParameters(0xFF); // back to RX max -- real length comes from GetRxBufferStatus
  startRX();
}

static void configureRadio()
{
  Serial.println();
  Serial.println("========== SX1262 CONFIG ==========");

  setRegulatorModeDCDC();
  // Chip is already in default STDBY_RC immediately after reset, so
  // TCXO setup + calibration happen here BEFORE requesting STDBY_XOSC --
  // asking the chip to switch to the crystal/TCXO clock before it even
  // knows the TCXO exists (voltage/startup delay) is out of datasheet
  // order and was a bug here previously.
  setDio3AsTcxoCtrl();
  calibrate();
  calibrateImage863to870();
  setStandby();
  setPacketTypeLoRa();
  setSyncWordPublic();
  setFrequency(LORA_FREQ);
  setModulation();
  setPacketParameters(0xFF); // RX default: max buffer, actual length read via GetRxBufferStatus
  setBufferBase();
  setPAConfig();
  setTxParams();
  enableDio2RF();
  setIRQ();
  clearIRQ();

  Serial.println("[SX1262] Frequency : 865.2325 MHz");
  Serial.println("[SX1262] SF        : 7");
  Serial.println("[SX1262] BW        : 125 kHz");
  Serial.println("[SX1262] CR        : 4/5");
  Serial.println("[SX1262] TX Power  : 14 dBm");
  Serial.println("====================================");
}

// ================================================================
//                       DOWNLINK HANDLING
// ================================================================
//
// Protocol: 1 byte command + payload
//   0x01 TIME_SYNC : 4 bytes big-endian Unix epoch seconds
//   0x02 RFID_ACK  : 4 bytes big-endian rfid row id (rfids.id), sent by
//                    the server in response to a welderlogin uplink
// ================================================================

static void printDownlinkHex(const uint8_t *payload, uint8_t length)
{
  // Raw byte dump of every downlink, before it's interpreted -- lets us
  // see exactly what arrived over RF (e.g. to cross-check against the
  // base64 phyPayload the network server/gateway bridge logged for this
  // same downlink) even if handleDownlink() below doesn't recognize it.
  Serial.print("[LoRa] Downlink raw bytes (hex): ");
  for (uint8_t i = 0; i < length; i++)
  {
    if (payload[i] < 0x10) Serial.print("0");
    Serial.print(payload[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

static uint32_t readUint32BE(uint8_t *payload)
{
  return ((uint32_t)payload[1] << 24) |
         ((uint32_t)payload[2] << 16) |
         ((uint32_t)payload[3] << 8)  |
          (uint32_t)payload[4];
}

static void handleDownlink(uint8_t *payload, uint8_t length)
{
  if (length >= 5 && payload[0] == 0x01)
  {
    uint32_t epoch = readUint32BE(payload);

    setEpoch(epoch);

    Serial.print("[TIME SYNC] Epoch set to: ");
    Serial.println(epoch);

    Serial.print("[TIME SYNC] Current time: ");
    Serial.println(getTimestamp());
  }
  else if (length >= 5 && payload[0] == 0x02)
  {
    currentRfidId = readUint32BE(payload);

    Serial.print("[RFID ACK] rfid id set to: ");
    Serial.println(currentRfidId);
  }
  else
  {
    Serial.print("[LoRa] Unknown downlink, length=");
    Serial.println(length);
  }
}

// ================================================================
//                       PUBLIC API
// ================================================================

void initRadio()
{
  pinMode(LORA_TXEN, OUTPUT);
  pinMode(LORA_RXEN, OUTPUT);
  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, LOW);

  pinMode(LORA_CS, OUTPUT);
  digitalWrite(LORA_CS, HIGH);

  pinMode(LORA_BUSY, INPUT);
  pinMode(LORA_DIO1, INPUT);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);

  Serial.println();
  Serial.println("========== LORA HSPI INIT ==========");

  radioSPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  Serial.println("[LoRa] HSPI started");
  Serial.print("[LoRa] SCK  = "); Serial.println(LORA_SCK);
  Serial.print("[LoRa] MISO = "); Serial.println(LORA_MISO);
  Serial.print("[LoRa] MOSI = "); Serial.println(LORA_MOSI);
  Serial.print("[LoRa] CS   = "); Serial.println(LORA_CS);

  radioReset();
  configureRadio();
  delay(200);

  digitalWrite(LORA_TXEN, LOW);
  digitalWrite(LORA_RXEN, HIGH);
  delay(2);

  startRX();
}

bool sendLoRa(const String &payload)
{
  Serial.println();
  Serial.println("---------------- LORA TX ----------------");
  Serial.println("[LoRa] TX Payload:");
  Serial.println(payload);

  digitalWrite(LORA_RXEN, LOW);
  digitalWrite(LORA_TXEN, HIGH);
  delay(2);

  setStandby();
  clearIRQ();
  setPacketParameters((uint8_t)payload.length()); // real length for THIS packet -- see note above
  writePayload(payload);
  startTX();

  unsigned long start = millis();
  unsigned long lastStatusPrint = 0;

  while (millis() - start < 5000)
  {
    uint16_t irq = getIRQ();

    if (millis() - lastStatusPrint > 500)
    {
      lastStatusPrint = millis();
      Serial.print("[LoRa] chip mode while waiting = ");
      Serial.println(getStatus());
    }

    if (irq & IRQ_TX_DONE)
    {
      clearIRQ();
      resumeListening();

      Serial.println("[LoRa] TX DONE");
      Serial.println("------------------------------------------");
      return true;
    }

    if (irq & IRQ_TIMEOUT)
    {
      clearIRQ();
      resumeListening();

      Serial.println("[LoRa] TX TIMEOUT");
      return false;
    }

    delay(5);
  }

  resumeListening();
  Serial.println("[LoRa] TX WAIT TIMEOUT");
  return false;
}

void checkAndProcessDownlink()
{
  uint16_t irq = getIRQ();

  if (irq & IRQ_RX_DONE)
  {
    uint8_t payloadLength = 0;
    uint8_t startPointer  = 0;

    getRxBufferStatus(payloadLength, startPointer);

    uint8_t buffer[32];
    if (payloadLength > sizeof(buffer))
    {
      payloadLength = sizeof(buffer);
    }

    readBuffer(startPointer, buffer, payloadLength);
    clearIRQ();

    Serial.print("[LoRa] Downlink received, length=");
    Serial.println(payloadLength);

    printDownlinkHex(buffer, payloadLength);
    handleDownlink(buffer, payloadLength);

    resumeListening();
  }

  if (irq & IRQ_CRC_ERROR)
  {
    clearIRQ();
  }
}
