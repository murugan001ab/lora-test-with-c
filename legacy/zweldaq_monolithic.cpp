#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <Adafruit_ADS1X15.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>
#include <esp_sleep.h>

// ================================================================
//                         DEVICE CONFIG
// ================================================================

#define DEVICE_ID        1
#define ORGANIZATION_ID  1

// ================================================================
//                         SENSOR PINS
// ================================================================

#define Voltage_Sens     39
#define FeedRate_Sens    34
#define GasFlow_Sens     35

// ================================================================
//                         RFID PINS
// ================================================================
// RFID uses ESP32 VSPI peripheral through global SPI object
//
// SCK  = GPIO14
// MISO = GPIO12
// MOSI = GPIO13
// SS   = GPIO15
// RST  = GPIO4
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

HardwareSerial DWINSerial(2);

// ================================================================
//                         LORA PINS
// ================================================================
// LoRa uses HSPI
//
// SCK  = GPIO18
// MISO = GPIO19
// MOSI = GPIO23
// CS   = GPIO5
// BUSY = GPIO25
// DIO1 = GPIO26
// RST  = GPIO27
// ================================================================

#define LORA_SCK         18
#define LORA_MISO        19
#define LORA_MOSI        23
#define LORA_CS          5
#define LORA_RST         27
#define LORA_BUSY        25
#define LORA_DIO1        26

// EBYTE RF switch
#define LORA_RXEN        32
#define LORA_TXEN        33

#define LED_PIN          2

// Graceful shutdown button (ESP32 devkit BOOT button, active LOW)
#define SHUTDOWN_BTN_PIN 0

const unsigned long SHUTDOWN_HOLD_MS = 2000;

unsigned long shutdownPressStart = 0;
bool shutdownTriggered = false;

// ================================================================
//                         SPI BUSES
// ================================================================

// RFID:
// Uses global SPI object = ESP32 VSPI peripheral
//
// IMPORTANT:
// Your MFRC522 library does NOT support SPIClass constructor.
// Therefore we use the normal:
//
// MFRC522 mfrc522(SS, RST);
//
// and configure global SPI with RFID pins.

// LoRa:
// Uses HSPI peripheral separately
SPIClass radioSPI(HSPI);

SPISettings radioSPISettings(
  1000000,
  MSBFIRST,
  SPI_MODE0
);

// ================================================================
//                         RFID OBJECT
// ================================================================

MFRC522 mfrc522(
  RFID_SS_PIN,
  RFID_RST_PIN
);

// ================================================================
//                         ADS1115
// ================================================================

Adafruit_ADS1115 ads;

// ================================================================
//                         CALIBRATION
// ================================================================

#define Ct_Cal_Factor    0.01855678
#define Vg_Cal_Factor    0.0245250376
#define FR_Cal_Factor    0.032334515
#define GF_Cal_Factor    0.0122189638

#define ref_volt         1.5
#define ref_volt1        3.30

float Cal_Factor =
  ref_volt / 3.3;

float Cal_Factor1 =
  ref_volt1 / 3.0;

float Cal_Voltage  = 0.0;
float Cal_Current  = 0.0;
float Cal_FeedRate = 0.0;
float Cal_GasFlow  = 0.0;

int16_t rawADS = 0;

// ================================================================
//                       LORA PARAMETERS
// ================================================================

#define LORA_FREQ        865232500UL
#define LORA_SF          7
#define LORA_BW          125000
#define LORA_CR          1
#define TX_POWER         14

#define FREQ_STEP \
  (32000000.0 / 33554432.0)

// ================================================================
//                       LORA COMMANDS
// ================================================================

#define CMD_STANDBY            0x80
#define CMD_SET_PACKET_TYPE    0x8A
#define CMD_SET_RF_FREQ        0x86
#define CMD_SET_MODULATION     0x8B
#define CMD_SET_PACKET         0x8C
#define CMD_SET_BUFFER         0x8F
#define CMD_WRITE_BUFFER       0x0E
#define CMD_READ_BUFFER        0x1E
#define CMD_GET_RX_BUFFER_STATUS 0x13
#define CMD_SET_TX             0x83
#define CMD_SET_RX             0x82
#define CMD_GET_IRQ            0x12
#define CMD_CLEAR_IRQ          0x02
#define CMD_SET_IRQ            0x08
#define CMD_SET_TX_PARAMS      0x8E
#define CMD_SET_PA_CONFIG      0x95
#define CMD_DIO2_RF_SWITCH     0x9D

#define IRQ_TX_DONE            0x0001
#define IRQ_RX_DONE            0x0002
#define IRQ_TIMEOUT            0x0200
#define IRQ_CRC_ERROR          0x0040

// ================================================================
//                       SYSTEM STATE
// ================================================================

enum SystemState
{
  IDLE,
  LOGIN_PENDING,
  LOGGED_IN,
  LOGOUT_PENDING
};

SystemState systemState =
  IDLE;

bool loggedIn =
  false;

bool weldingStarted =
  false;

String currentRFID = "";
String sessionID   = "";

unsigned long loginDebounceTime =
  0;

unsigned long lastWeldDataTime =
  0;

const unsigned long RFID_DEBOUNCE =
  800;

const unsigned long WELD_DATA_INTERVAL =
  3000;

// ================================================================
//                         TIME
// ================================================================

unsigned long baseMillis = 0;

unsigned long epochBase = 0;

// ================================================================
//                     TIMESTAMP
// ================================================================

String getTimestamp()
{
  unsigned long elapsedSeconds =
    (millis() - baseMillis) / 1000UL;

  unsigned long currentEpoch =
    epochBase + elapsedSeconds;

  unsigned long days =
    currentEpoch / 86400UL;

  unsigned long secondsOfDay =
    currentEpoch % 86400UL;

  int hour =
    secondsOfDay / 3600;

  int minute =
    (secondsOfDay % 3600) / 60;

  int second =
    secondsOfDay % 60;

  int year = 1970;

  unsigned long remainingDays =
    days;

  while (true)
  {
    bool leap =
      ((year % 4 == 0 &&
        year % 100 != 0) ||
       (year % 400 == 0));

    int daysInYear =
      leap ? 366 : 365;

    if (
      remainingDays >=
      (unsigned long)daysInYear
    )
    {
      remainingDays -=
        daysInYear;

      year++;
    }
    else
    {
      break;
    }
  }

  int monthDays[12] =
  {
    31, 28, 31, 30,
    31, 30, 31, 31,
    30, 31, 30, 31
  };

  bool leap =
    ((year % 4 == 0 &&
      year % 100 != 0) ||
     (year % 400 == 0));

  if (leap)
  {
    monthDays[1] = 29;
  }

  int month = 0;

  while (
    remainingDays >=
    (unsigned long)monthDays[month]
  )
  {
    remainingDays -=
      monthDays[month];

    month++;
  }

  month++;

  char buffer[40];

  sprintf(
    buffer,
    "%04d-%02d-%02dT%02d:%02d:%02dZ",
    year,
    month,
    (int)remainingDays + 1,
    hour,
    minute,
    second
  );

  return String(buffer);
}

// ================================================================
//                     UUID GENERATOR
// ================================================================

String generateUUID()
{
  uint32_t r1 = esp_random();
  uint32_t r2 = esp_random();
  uint32_t r3 = esp_random();
  uint32_t r4 = esp_random();
  uint32_t r5 = esp_random();

  char uuid[50];

  sprintf(
    uuid,
    "%08lX-%04lX-%04lX-%04lX-%08lX",
    (unsigned long)r1,
    (unsigned long)(r2 & 0xFFFF),
    (unsigned long)((r3 & 0x0FFF) | 0x4000),
    (unsigned long)((r4 & 0x3FFF) | 0x8000),
    (unsigned long)r5
  );

  return String(uuid);
}

// ================================================================
//                         LORA BUSY
// ================================================================

void waitBusy()
{
  unsigned long start =
    millis();

  while (
    digitalRead(LORA_BUSY) == HIGH
  )
  {
    if (
      millis() - start > 1000
    )
    {
      Serial.println(
        "[SX1262] ERROR: BUSY pin timeout!"
      );

      return;
    }

    delay(1);
  }
}

// ================================================================
//                         LORA CS
// ================================================================

void loraSelect()
{
  digitalWrite(
    LORA_CS,
    LOW
  );
}

void loraDeselect()
{
  digitalWrite(
    LORA_CS,
    HIGH
  );
}

// ================================================================
//                     LORA WRITE COMMAND
// ================================================================

void writeCommand(
  uint8_t command,
  const uint8_t *data,
  uint8_t length
)
{
  waitBusy();

  radioSPI.beginTransaction(
    radioSPISettings
  );

  loraSelect();

  radioSPI.transfer(
    command
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  )
  {
    radioSPI.transfer(
      data[i]
    );
  }

  loraDeselect();

  radioSPI.endTransaction();

  waitBusy();
}

// ================================================================
//                     LORA READ COMMAND
// ================================================================

void readCommand(
  uint8_t command,
  uint8_t *data,
  uint8_t length
)
{
  waitBusy();

  radioSPI.beginTransaction(
    radioSPISettings
  );

  loraSelect();

  radioSPI.transfer(
    command
  );

  radioSPI.transfer(
    0x00
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  )
  {
    data[i] =
      radioSPI.transfer(
        0x00
      );
  }

  loraDeselect();

  radioSPI.endTransaction();
}

// ================================================================
//                     RX BUFFER STATUS
// ================================================================

void getRxBufferStatus(
  uint8_t &payloadLength,
  uint8_t &startPointer
)
{
  uint8_t data[2];

  readCommand(
    CMD_GET_RX_BUFFER_STATUS,
    data,
    2
  );

  payloadLength = data[0];
  startPointer  = data[1];
}

// ================================================================
//                     READ LORA BUFFER
// ================================================================

void readBuffer(
  uint8_t offset,
  uint8_t *buffer,
  uint8_t length
)
{
  waitBusy();

  radioSPI.beginTransaction(
    radioSPISettings
  );

  loraSelect();

  radioSPI.transfer(
    CMD_READ_BUFFER
  );

  radioSPI.transfer(
    offset
  );

  radioSPI.transfer(
    0x00
  );

  for (
    uint8_t i = 0;
    i < length;
    i++
  )
  {
    buffer[i] =
      radioSPI.transfer(
        0x00
      );
  }

  loraDeselect();

  radioSPI.endTransaction();
}

// ================================================================
//                     SET STANDBY
// ================================================================

void setStandby()
{
  uint8_t data =
    0x01;

  writeCommand(
    CMD_STANDBY,
    &data,
    1
  );
}

// ================================================================
//                     RADIO RESET
// ================================================================

void radioReset()
{
  Serial.println(
    "[SX1262] Resetting..."
  );

  digitalWrite(
    LORA_RST,
    LOW
  );

  delay(20);

  digitalWrite(
    LORA_RST,
    HIGH
  );

  delay(100);

  waitBusy();

  Serial.println(
    "[SX1262] Reset complete"
  );
}

// ================================================================
//                     PACKET TYPE
// ================================================================

void setPacketTypeLoRa()
{
  uint8_t data[1] =
  {
    0x01
  };

  writeCommand(
    CMD_SET_PACKET_TYPE,
    data,
    1
  );
}

// ================================================================
//                     FREQUENCY
// ================================================================

void setFrequency(
  uint32_t frequency
)
{
  uint32_t frf =
    (uint32_t)
    (
      ((double)frequency /
       FREQ_STEP)
    );

  uint8_t data[4];

  data[0] =
    (frf >> 24) & 0xFF;

  data[1] =
    (frf >> 16) & 0xFF;

  data[2] =
    (frf >> 8) & 0xFF;

  data[3] =
    frf & 0xFF;

  writeCommand(
    CMD_SET_RF_FREQ,
    data,
    4
  );
}

// ================================================================
//                     MODULATION
// ================================================================

void setModulation()
{
  uint8_t data[4];

  data[0] =
    LORA_SF;

  // BW 125 kHz
  data[1] =
    0x04;

  // CR 4/5
  data[2] =
    LORA_CR;

  data[3] =
    0x00;

  writeCommand(
    CMD_SET_MODULATION,
    data,
    4
  );
}

// ================================================================
//                     PACKET PARAMETERS
// ================================================================

void setPacketParameters()
{
  uint8_t data[6];

  // Preamble = 8
  data[0] =
    0x00;

  data[1] =
    0x08;

  // Explicit header
  data[2] =
    0x00;

  // Variable payload
  data[3] =
    0xFF;

  // CRC ON
  data[4] =
    0x01;

  // Normal IQ
  data[5] =
    0x00;

  writeCommand(
    CMD_SET_PACKET,
    data,
    6
  );
}

// ================================================================
//                     BUFFER BASE
// ================================================================

void setBufferBase()
{
  uint8_t data[2] =
  {
    0x00,
    0x00
  };

  writeCommand(
    CMD_SET_BUFFER,
    data,
    2
  );
}

// ================================================================
//                     TX PARAMETERS
// ================================================================

void setTxParams()
{
  uint8_t data[2];

  data[0] =
    TX_POWER;

  data[1] =
    0x04;

  writeCommand(
    CMD_SET_TX_PARAMS,
    data,
    2
  );
}

// ================================================================
//                     PA CONFIG
// ================================================================

void setPAConfig()
{
  uint8_t data[4];

  data[0] =
    0x04;

  data[1] =
    0x07;

  data[2] =
    0x00;

  data[3] =
    0x01;

  writeCommand(
    CMD_SET_PA_CONFIG,
    data,
    4
  );
}

// ================================================================
//                     DIO2 RF SWITCH
// ================================================================

void enableDio2RF()
{
  uint8_t data =
    0x01;

  writeCommand(
    CMD_DIO2_RF_SWITCH,
    &data,
    1
  );
}

// ================================================================
//                     SET IRQ
// ================================================================

void setIRQ()
{
  uint8_t data[8];

  uint16_t irqMask =
    IRQ_TX_DONE |
    IRQ_RX_DONE |
    IRQ_TIMEOUT |
    IRQ_CRC_ERROR;

  data[0] =
    (irqMask >> 8) & 0xFF;

  data[1] =
    irqMask & 0xFF;

  data[2] =
    data[0];

  data[3] =
    data[1];

  data[4] =
    0x00;

  data[5] =
    0x00;

  data[6] =
    0x00;

  data[7] =
    0x00;

  writeCommand(
    CMD_SET_IRQ,
    data,
    8
  );
}

// ================================================================
//                     CLEAR IRQ
// ================================================================

void clearIRQ()
{
  uint8_t data[2] =
  {
    0xFF,
    0xFF
  };

  writeCommand(
    CMD_CLEAR_IRQ,
    data,
    2
  );
}

// ================================================================
//                     GET IRQ
// ================================================================

uint16_t getIRQ()
{
  uint8_t data[2];

  readCommand(
    CMD_GET_IRQ,
    data,
    2
  );

  return
    ((uint16_t)data[0] << 8) |
    data[1];
}

// ================================================================
//                     WRITE LORA PAYLOAD
// ================================================================

void writePayload(
  const String &payload
)
{
  waitBusy();

  radioSPI.beginTransaction(
    radioSPISettings
  );

  loraSelect();

  radioSPI.transfer(
    CMD_WRITE_BUFFER
  );

  radioSPI.transfer(
    0x00
  );

  for (
    size_t i = 0;
    i < payload.length();
    i++
  )
  {
    radioSPI.transfer(
      (uint8_t)payload[i]
    );
  }

  loraDeselect();

  radioSPI.endTransaction();

  waitBusy();
}

// ================================================================
//                     START TX
// ================================================================

void startTX()
{
  uint8_t data[3] =
  {
    0x00,
    0x00,
    0x00
  };

  writeCommand(
    CMD_SET_TX,
    data,
    3
  );
}

// ================================================================
//                     START RX
// ================================================================

void startRX()
{
  uint8_t data[3] =
  {
    0xFF,
    0xFF,
    0xFF
  };

  writeCommand(
    CMD_SET_RX,
    data,
    3
  );
}

// ================================================================
//                     RESUME LISTENING
// ================================================================

void resumeListening()
{
  digitalWrite(
    LORA_TXEN,
    LOW
  );

  digitalWrite(
    LORA_RXEN,
    HIGH
  );

  delay(2);

  startRX();
}

// ================================================================
//                     LORA SEND
// ================================================================

bool sendLoRa(
  const String &payload
)
{
  Serial.println();
  Serial.println(
    "---------------- LORA TX ----------------"
  );

  Serial.println(
    "[LoRa] TX Payload:"
  );

  Serial.println(
    payload
  );

  // TX switch
  digitalWrite(
    LORA_RXEN,
    LOW
  );

  digitalWrite(
    LORA_TXEN,
    HIGH
  );

  delay(2);

  setStandby();

  clearIRQ();

  writePayload(
    payload
  );

  startTX();

  unsigned long start =
    millis();

  while (
    millis() - start < 5000
  )
  {
    uint16_t irq =
      getIRQ();

    if (
      irq & IRQ_TX_DONE
    )
    {
      clearIRQ();

      resumeListening();

      Serial.println(
        "[LoRa] TX DONE"
      );

      Serial.println(
        "------------------------------------------"
      );

      return true;
    }

    if (
      irq & IRQ_TIMEOUT
    )
    {
      clearIRQ();

      resumeListening();

      Serial.println(
        "[LoRa] TX TIMEOUT"
      );

      return false;
    }

    delay(5);
  }

  resumeListening();

  Serial.println(
    "[LoRa] TX WAIT TIMEOUT"
  );

  return false;
}

// ================================================================
//                     CONFIGURE LORA
// ================================================================

void configureRadio()
{
  Serial.println();
  Serial.println(
    "========== SX1262 CONFIG =========="
  );

  setStandby();

  setPacketTypeLoRa();

  setFrequency(
    LORA_FREQ
  );

  setModulation();

  setPacketParameters();

  setBufferBase();

  setPAConfig();

  setTxParams();

  enableDio2RF();

  setIRQ();

  clearIRQ();

  Serial.println(
    "[SX1262] Frequency : 865.2325 MHz"
  );

  Serial.println(
    "[SX1262] SF        : 7"
  );

  Serial.println(
    "[SX1262] BW        : 125 kHz"
  );

  Serial.println(
    "[SX1262] CR        : 4/5"
  );

  Serial.println(
    "[SX1262] TX Power  : 14 dBm"
  );

  Serial.println(
    "===================================="
  );
}

// ================================================================
//                     HANDLE DOWNLINK PAYLOAD
// ================================================================
//
// Protocol: 1 byte command + payload
//   0x01 TIME_SYNC : 4 bytes big-endian Unix epoch seconds
// ================================================================

void handleDownlink(
  uint8_t *payload,
  uint8_t length
)
{
  if (
    length >= 5 &&
    payload[0] == 0x01
  )
  {
    uint32_t epoch =
      ((uint32_t)payload[1] << 24) |
      ((uint32_t)payload[2] << 16) |
      ((uint32_t)payload[3] << 8)  |
       (uint32_t)payload[4];

    baseMillis =
      millis();

    epochBase =
      epoch;

    Serial.print(
      "[TIME SYNC] Epoch set to: "
    );

    Serial.println(
      epoch
    );

    Serial.print(
      "[TIME SYNC] Current time: "
    );

    Serial.println(
      getTimestamp()
    );
  }
  else
  {
    Serial.print(
      "[LoRa] Unknown downlink, length="
    );

    Serial.println(
      length
    );
  }
}

// ================================================================
//                     CHECK FOR DOWNLINK
// ================================================================

void checkAndProcessDownlink()
{
  uint16_t irq =
    getIRQ();

  if (
    irq & IRQ_RX_DONE
  )
  {
    uint8_t payloadLength = 0;
    uint8_t startPointer  = 0;

    getRxBufferStatus(
      payloadLength,
      startPointer
    );

    uint8_t buffer[32];

    if (
      payloadLength > sizeof(buffer)
    )
    {
      payloadLength =
        sizeof(buffer);
    }

    readBuffer(
      startPointer,
      buffer,
      payloadLength
    );

    clearIRQ();

    Serial.print(
      "[LoRa] Downlink received, length="
    );

    Serial.println(
      payloadLength
    );

    handleDownlink(
      buffer,
      payloadLength
    );

    resumeListening();
  }

  if (
    irq & IRQ_CRC_ERROR
  )
  {
    clearIRQ();
  }
}

// ================================================================
//                     DWIN WRITE
// ================================================================

void DWIN_Write(
  uint16_t address,
  uint16_t value
)
{
  uint8_t frame[8];

  frame[0] =
    0x5A;

  frame[1] =
    0xA5;

  frame[2] =
    0x05;

  frame[3] =
    0x82;

  frame[4] =
    (address >> 8) & 0xFF;

  frame[5] =
    address & 0xFF;

  frame[6] =
    (value >> 8) & 0xFF;

  frame[7] =
    value & 0xFF;

  DWINSerial.write(
    frame,
    8
  );
}

// ================================================================
//                     DWIN PAGE
// ================================================================

void DWIN_ChangePage(
  uint8_t page
)
{
  DWIN_Write(
    0x0084,
    page
  );
}

// ================================================================
//                     PRINT SENSOR READINGS
// ================================================================

void printSensorReadings()
{
  Serial.print("Voltage: ");
  Serial.print(Cal_Voltage, 2);
  Serial.print(" V | Current: ");
  Serial.print(Cal_Current, 2);
  Serial.print(" A | FeedRate: ");
  Serial.print(Cal_FeedRate, 2);
  Serial.print(" | GasFlow: ");
  Serial.println(Cal_GasFlow, 2);
}

// ================================================================
//                     SENSOR READ
// ================================================================

void readSensors()
{
  // --------------------------------------------------------------
  // ADS1115 CURRENT
  // --------------------------------------------------------------

  // --------------------------------------------------------------
  // ANALOG + ADS1115 AVERAGING
  // --------------------------------------------------------------

  long voltageSum = 0;
  long feedSum    = 0;
  long gasSum     = 0;
  long currentSum = 0;

  const int sampleCount =
    200;

  for (
    int i = 0;
    i < sampleCount;
    i++
  )
  {
    voltageSum +=
      analogRead(
        Voltage_Sens
      );

    feedSum +=
      analogRead(
        FeedRate_Sens
      );

    gasSum +=
      analogRead(
        GasFlow_Sens
      );

    currentSum +=
      ads.readADC_SingleEnded(0);

    delayMicroseconds(
      200
    );
  }

  float RawADCVoltage =
    voltageSum /
    (float)sampleCount;

  float RawADCFeedRate =
    feedSum /
    (float)sampleCount;

  float RawADCGasFlow =
    gasSum /
    (float)sampleCount;

  rawADS =
    (int16_t)(currentSum / sampleCount);

  // --------------------------------------------------------------
  // CALCULATIONS
  // --------------------------------------------------------------

  Cal_Voltage =
    RawADCVoltage *
    Vg_Cal_Factor *
    Cal_Factor;

  Cal_Current =
    rawADS *
    Ct_Cal_Factor;

  Cal_FeedRate =
    RawADCFeedRate *
    FR_Cal_Factor *
    Cal_Factor;

  Cal_GasFlow =
    RawADCGasFlow *
    GF_Cal_Factor *
    Cal_Factor;

  // --------------------------------------------------------------
  // DWIN
  // --------------------------------------------------------------

  DWIN_Write(
    0x1220,
    (uint16_t)Cal_Voltage
  );

  DWIN_Write(
    0x1240,
    (uint16_t)Cal_Current
  );

  DWIN_Write(
    0x1260,
    (uint16_t)Cal_FeedRate
  );

  DWIN_Write(
    0x1280,
    (uint16_t)Cal_GasFlow
  );

  // --------------------------------------------------------------
  // SERIAL SENSOR OUTPUT
  // --------------------------------------------------------------

  printSensorReadings();
}

// ================================================================
//                     RFID INIT
// ================================================================

void RFID_Init()
{
  Serial.println();
  Serial.println(
    "========== RFID VSPI INIT =========="
  );

  pinMode(
    RFID_SS_PIN,
    OUTPUT
  );

  digitalWrite(
    RFID_SS_PIN,
    HIGH
  );

  pinMode(
    RFID_RST_PIN,
    OUTPUT
  );

  digitalWrite(
    RFID_RST_PIN,
    HIGH
  );

  // ============================================================
  // IMPORTANT
  //
  // Old MFRC522 library uses global SPI object.
  //
  // On ESP32, global SPI = VSPI.
  //
  // We configure VSPI with RFID pins.
  // ============================================================

  SPI.begin(
    RFID_SCK_PIN,
    RFID_MISO_PIN,
    RFID_MOSI_PIN,
    RFID_SS_PIN
  );

  delay(100);

  mfrc522.PCD_Init();

  delay(100);

  byte version =
    mfrc522.PCD_ReadRegister(
      mfrc522.VersionReg
    );

  Serial.print(
    "[RFID] Version: 0x"
  );

  Serial.println(
    version,
    HEX
  );

  if (
    version == 0x00 ||
    version == 0xFF
  )
  {
    Serial.println(
      "[RFID] ERROR - MFRC522 NOT DETECTED"
    );
  }
  else
  {
    Serial.println(
      "[RFID] MFRC522 OK"
    );
  }

  Serial.println(
    "===================================="
  );
}

// ================================================================
//                     RFID READ
// ================================================================

String readRFID()
{
  if (
    !mfrc522.PICC_IsNewCardPresent()
  )
  {
    return "";
  }

  if (
    !mfrc522.PICC_ReadCardSerial()
  )
  {
    return "";
  }

  String uid = "";

  for (
    byte i = 0;
    i < mfrc522.uid.size;
    i++
  )
  {
    if (
      mfrc522.uid.uidByte[i] < 0x10
    )
    {
      uid += "0";
    }

    uid += String(
      mfrc522.uid.uidByte[i],
      HEX
    );
  }

  uid.toLowerCase();

  mfrc522.PICC_HaltA();

  mfrc522.PCD_StopCrypto1();

  return uid;
}

// ================================================================
//                     PRINT PAYLOAD
// ================================================================

void printPayload(
  const String &payload
)
{
  Serial.println();
  Serial.println(
    "================================================"
  );

  Serial.println(
    "                 JSON PAYLOAD"
  );

  Serial.println(
    "================================================"
  );

  Serial.println(
    payload
  );

  Serial.println(
    "================================================"
  );

  Serial.println();
}

// ================================================================
//                     DEVICE STATUS PAYLOAD
// ================================================================

void sendDeviceOnline()
{
  StaticJsonDocument<256> doc;

  doc["topic"] =
    "device_status";

  doc["deviceId"] =
    DEVICE_ID;

  doc["status"] =
    "online";

  doc["reason"] =
    "device_connect";

  doc["timestamp"] =
    getTimestamp();

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] DEVICE STATUS"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     DEVICE OFFLINE PAYLOAD
// ================================================================

void sendDeviceOffline()
{
  StaticJsonDocument<256> doc;

  doc["topic"] =
    "device_status";

  doc["deviceId"] =
    DEVICE_ID;

  doc["status"] =
    "offline";

  doc["reason"] =
    "device_disconnect";

  doc["timestamp"] =
    getTimestamp();

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] DEVICE OFFLINE"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     RFID LOGIN PAYLOAD
// ================================================================

void sendLogin(
  String rfid
)
{
  StaticJsonDocument<256> doc;

  doc["topic"] =
    "welderlogin";

  doc["deviceId"] =
    DEVICE_ID;

  doc["rfid"] =
    rfid;

  doc["timestamp"] =
    getTimestamp();

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] RFID LOGIN"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     RFID LOGOUT PAYLOAD
// ================================================================

void sendLogout(
  String rfid
)
{
  StaticJsonDocument<256> doc;

  doc["topic"] =
    "welderlogout";

  doc["deviceId"] =
    DEVICE_ID;

  doc["rfid"] =
    rfid;

  doc["timestamp"] =
    getTimestamp();

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] RFID LOGOUT"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     WELDER START PAYLOAD
// ================================================================

void sendWeldStart()
{
  StaticJsonDocument<384> doc;

  doc["topic"] =
    "welder_start";

  doc["sessionId"] =
    sessionID;

  doc["deviceId"] =
    String(DEVICE_ID);

  doc["time"] =
    getTimestamp();

  doc["organization_id"] =
    ORGANIZATION_ID;

  doc["rfid"] =
    currentRFID;

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] WELDER START"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     WELDER DATA PAYLOAD
// ================================================================

void sendWeldData()
{
  StaticJsonDocument<384> doc;

  doc["topic"] =
    "welder_data";

  doc["sessionId"] =
    sessionID;

  doc["deviceId"] =
    DEVICE_ID;

  doc["current"] =
    Cal_Current;

  doc["voltage"] =
    Cal_Voltage;

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] WELDER DATA"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     WELDER STOP PAYLOAD
// ================================================================

void sendWeldStop()
{
  StaticJsonDocument<256> doc;

  doc["topic"] =
    "welder_stop";

  doc["sessionId"] =
    sessionID;

  doc["stop_time"] =
    getTimestamp();

  doc["deviceId"] =
    String(DEVICE_ID);

  String payload;

  serializeJson(
    doc,
    payload
  );

  Serial.println(
    "[PAYLOAD] WELDER STOP"
  );

  printPayload(
    payload
  );

  sendLoRa(
    payload
  );
}

// ================================================================
//                     RFID HANDLER
// ================================================================

void handleRFID()
{
  String detectedRFID =
    readRFID();

  if (
    detectedRFID.length() == 0
  )
  {
    return;
  }

  if (
    millis() - loginDebounceTime <
    RFID_DEBOUNCE
  )
  {
    return;
  }

  loginDebounceTime =
    millis();

  Serial.println();
  Serial.println(
    "******** RFID CARD DETECTED ********"
  );

  Serial.print(
    "RFID UID: "
  );

  Serial.println(
    detectedRFID
  );

  // ============================================================
  // LOGIN
  // ============================================================

  if (!loggedIn)
  {
    currentRFID =
      detectedRFID;

    sessionID =
      generateUUID();

    loggedIn =
      true;

    systemState =
      LOGGED_IN;

    weldingStarted =
      false;

    sendLogin(
      currentRFID
    );

    Serial.println(
      "[SYSTEM] OPERATOR LOGGED IN"
    );

    Serial.print(
      "[SYSTEM] Session ID: "
    );

    Serial.println(
      sessionID
    );
  }

  // ============================================================
  // LOGOUT
  // ============================================================

  else
  {
    if (
      weldingStarted
    )
    {
      Serial.println(
        "[RFID] Cannot logout while welding"
      );

      return;
    }

    sendLogout(
      currentRFID
    );

    loggedIn =
      false;

    weldingStarted =
      false;

    currentRFID =
      "";

    sessionID =
      "";

    systemState =
      IDLE;

    Serial.println(
      "[SYSTEM] OPERATOR LOGGED OUT"
    );
  }
}

// ================================================================
//                         SETUP
// ================================================================

void setup()
{
  Serial.begin(
    115200
  );

  delay(1000);

  Serial.println();
  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "          ZWELDAQ ESP32"
  );

  Serial.println(
    " RFID + ADS1115 + DWIN + SX1262"
  );

  Serial.println(
    "========================================"
  );

  // --------------------------------------------------------------
  // TIME
  // --------------------------------------------------------------
  // epochBase starts at 0 (1970-01-01) until the server sends a
  // TIME_SYNC downlink after the device_connect status is received;
  // see handleDownlink(). Timestamps before that sync will read as
  // 1970 epoch time.
  // --------------------------------------------------------------

  baseMillis =
    millis();

  epochBase =
    0;

  // --------------------------------------------------------------
  // LED
  // --------------------------------------------------------------

  pinMode(
    LED_PIN,
    OUTPUT
  );

  digitalWrite(
    LED_PIN,
    LOW
  );

  // --------------------------------------------------------------
  // SHUTDOWN BUTTON
  // --------------------------------------------------------------

  pinMode(
    SHUTDOWN_BTN_PIN,
    INPUT_PULLUP
  );

  // --------------------------------------------------------------
  // LORA RF SWITCH
  // --------------------------------------------------------------

  pinMode(
    LORA_TXEN,
    OUTPUT
  );

  pinMode(
    LORA_RXEN,
    OUTPUT
  );

  digitalWrite(
    LORA_TXEN,
    LOW
  );

  digitalWrite(
    LORA_RXEN,
    LOW
  );

  // --------------------------------------------------------------
  // LORA PINS
  // --------------------------------------------------------------

  pinMode(
    LORA_CS,
    OUTPUT
  );

  digitalWrite(
    LORA_CS,
    HIGH
  );

  pinMode(
    LORA_BUSY,
    INPUT
  );

  pinMode(
    LORA_DIO1,
    INPUT
  );

  pinMode(
    LORA_RST,
    OUTPUT
  );

  digitalWrite(
    LORA_RST,
    HIGH
  );

  // --------------------------------------------------------------
  // RFID
  // --------------------------------------------------------------

  RFID_Init();

  // --------------------------------------------------------------
  // DWIN
  // --------------------------------------------------------------

  DWINSerial.begin(
    115200,
    SERIAL_8N1,
    DWIN_RX_PIN,
    DWIN_TX_PIN
  );

  delay(200);

  DWIN_ChangePage(
    0
  );

  Serial.println(
    "[DWIN] UART2 initialized"
  );

  // --------------------------------------------------------------
  // ADS1115
  // --------------------------------------------------------------

  Wire.begin();

  if (
    !ads.begin()
  )
  {
    Serial.println(
      "[ADS1115] ERROR - NOT FOUND"
    );
  }
  else
  {
    Serial.println(
      "[ADS1115] ADS1115 OK"
    );
  }

  ads.setGain(
    GAIN_TWOTHIRDS
  );

  // --------------------------------------------------------------
  // SENSOR PINS
  // --------------------------------------------------------------

  pinMode(
    Voltage_Sens,
    INPUT
  );

  pinMode(
    FeedRate_Sens,
    INPUT
  );

  pinMode(
    GasFlow_Sens,
    INPUT
  );

  // --------------------------------------------------------------
  // LORA HSPI
  // --------------------------------------------------------------

  Serial.println();
  Serial.println(
    "========== LORA HSPI INIT =========="
  );

  radioSPI.begin(
    LORA_SCK,
    LORA_MISO,
    LORA_MOSI,
    LORA_CS
  );

  Serial.println(
    "[LoRa] HSPI started"
  );

  Serial.print(
    "[LoRa] SCK  = "
  );

  Serial.println(
    LORA_SCK
  );

  Serial.print(
    "[LoRa] MISO = "
  );

  Serial.println(
    LORA_MISO
  );

  Serial.print(
    "[LoRa] MOSI = "
  );

  Serial.println(
    LORA_MOSI
  );

  Serial.print(
    "[LoRa] CS   = "
  );

  Serial.println(
    LORA_CS
  );

  // --------------------------------------------------------------
  // LORA RESET
  // --------------------------------------------------------------

  radioReset();

  // --------------------------------------------------------------
  // CONFIGURE LORA
  // --------------------------------------------------------------

  configureRadio();

  delay(200);

  // --------------------------------------------------------------
  // DEVICE ONLINE
  // --------------------------------------------------------------

  sendDeviceOnline();

  // --------------------------------------------------------------
  // RX
  // --------------------------------------------------------------

  digitalWrite(
    LORA_TXEN,
    LOW
  );

  digitalWrite(
    LORA_RXEN,
    HIGH
  );

  delay(2);

  startRX();

  // --------------------------------------------------------------
  // READY
  // --------------------------------------------------------------

  digitalWrite(
    LED_PIN,
    HIGH
  );

  Serial.println();
  Serial.println(
    "========================================"
  );

  Serial.println(
    "             SYSTEM READY"
  );

  Serial.println(
    "========================================"
  );

  Serial.println(
    "RFID  -> VSPI"
  );

  Serial.println(
    "LoRa  -> HSPI"
  );

  Serial.println(
    "ADS1115 -> I2C"
  );

  Serial.println(
    "DWIN -> UART2"
  );

  Serial.println(
    "========================================"
  );
}

// ================================================================
//                          LOOP
// ================================================================

void loop()
{
  // --------------------------------------------------------------
  // RFID
  // --------------------------------------------------------------

  handleRFID();

  // --------------------------------------------------------------
  // DOWNLINK
  // --------------------------------------------------------------

  checkAndProcessDownlink();

  // --------------------------------------------------------------
  // SENSOR
  // --------------------------------------------------------------

  readSensors();

  // --------------------------------------------------------------
  // WELDING START
  // --------------------------------------------------------------

  if (
    Cal_Current > 10.0 &&
    loggedIn &&
    !weldingStarted
  )
  {
    weldingStarted =
      true;

    lastWeldDataTime =
      millis();

    sendWeldStart();

    Serial.println(
      "[SYSTEM] WELDING STARTED"
    );
  }

  // --------------------------------------------------------------
  // WELDING DATA
  // --------------------------------------------------------------

  if (
    Cal_Current > 10.0 &&
    loggedIn &&
    weldingStarted
  )
  {
    if (
      millis() - lastWeldDataTime >=
      WELD_DATA_INTERVAL
    )
    {
      lastWeldDataTime =
        millis();

      sendWeldData();
    }
  }

  // --------------------------------------------------------------
  // WELDING STOP
  // --------------------------------------------------------------

  if (
    Cal_Current <= 10.0 &&
    weldingStarted
  )
  {
    sendWeldStop();

    weldingStarted =
      false;

    Serial.println(
      "[SYSTEM] WELDING STOPPED"
    );
  }

  // --------------------------------------------------------------
  // SHUTDOWN BUTTON
  // --------------------------------------------------------------

  if (
    digitalRead(SHUTDOWN_BTN_PIN) == LOW
  )
  {
    if (shutdownPressStart == 0)
    {
      shutdownPressStart =
        millis();
    }
    else if (
      !shutdownTriggered &&
      millis() - shutdownPressStart >= SHUTDOWN_HOLD_MS
    )
    {
      shutdownTriggered =
        true;

      Serial.println(
        "[SYSTEM] SHUTDOWN BUTTON HELD - SENDING OFFLINE STATUS"
      );

      sendDeviceOffline();

      Serial.println(
        "[SYSTEM] Entering deep sleep"
      );

      delay(200);

      esp_deep_sleep_start();
    }
  }
  else
  {
    shutdownPressStart =
      0;

    shutdownTriggered =
      false;
  }

  delay(100);
}