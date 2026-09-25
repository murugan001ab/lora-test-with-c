#include "rfid_handler.h"
#include "config.h"
#include "state.h"
#include "time_utils.h"
#include "payloads.h"

#include <SPI.h>
#include <MFRC522.h>

static MFRC522 mfrc522(RFID_SS_PIN, RFID_RST_PIN);

static String readRFID()
{
  if (!mfrc522.PICC_IsNewCardPresent())
  {
    return "";
  }

  if (!mfrc522.PICC_ReadCardSerial())
  {
    return "";
  }

  String uid = "";

  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    if (mfrc522.uid.uidByte[i] < 0x10)
    {
      uid += "0";
    }
    uid += String(mfrc522.uid.uidByte[i], HEX);
  }

  uid.toLowerCase();

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();

  return uid;
}

void RFID_Init()
{
  Serial.println();
  Serial.println("========== RFID VSPI INIT ==========");

  pinMode(RFID_SS_PIN, OUTPUT);
  digitalWrite(RFID_SS_PIN, HIGH);

  pinMode(RFID_RST_PIN, OUTPUT);
  digitalWrite(RFID_RST_PIN, HIGH);

  // The MFRC522 library uses the global SPI object; on ESP32 that's
  // VSPI. Configure VSPI with the RFID pins here.
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);

  delay(100);
  mfrc522.PCD_Init();
  delay(100);

  byte version = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);

  Serial.print("[RFID] Version: 0x");
  Serial.println(version, HEX);

  if (version == 0x00 || version == 0xFF)
  {
    Serial.println("[RFID] ERROR - MFRC522 NOT DETECTED");
  }
  else
  {
    Serial.println("[RFID] MFRC522 OK");
  }

  Serial.println("====================================");
}

void handleRFID()
{
  String detectedRFID = readRFID();

  if (detectedRFID.length() == 0)
  {
    return;
  }

  if (millis() - loginDebounceTime < RFID_DEBOUNCE)
  {
    return;
  }

  loginDebounceTime = millis();

  Serial.println();
  Serial.println("******** RFID CARD DETECTED ********");
  Serial.print("RFID UID: ");
  Serial.println(detectedRFID);

  if (!loggedIn)
  {
    // LOGIN
    currentRFID    = detectedRFID;
    sessionID      = generateUUID();
    loggedIn       = true;
    systemState    = LOGGED_IN;
    weldingStarted = false;

    sendLogin(currentRFID);

    Serial.println("[SYSTEM] OPERATOR LOGGED IN");
    Serial.print("[SYSTEM] Session ID: ");
    Serial.println(sessionID);
  }
  else
  {
    // LOGOUT
    if (weldingStarted)
    {
      Serial.println("[RFID] Cannot logout while welding");
      return;
    }

    sendLogout(currentRFID);

    loggedIn       = false;
    weldingStarted = false;
    currentRFID    = "";
    sessionID      = "";
    systemState    = IDLE;

    Serial.println("[SYSTEM] OPERATOR LOGGED OUT");
  }
}
