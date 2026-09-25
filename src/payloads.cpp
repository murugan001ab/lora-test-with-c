#include "payloads.h"
#include "config.h"
#include "state.h"
#include "time_utils.h"
#include "radio_sx1262.h"

#include <ArduinoJson.h>

void printPayload(const String &payload)
{
  Serial.println();
  Serial.println("================================================");
  Serial.println("                 JSON PAYLOAD");
  Serial.println("================================================");
  Serial.println(payload);
  Serial.println("================================================");
  Serial.println();
}

void sendDeviceOnline()
{
  StaticJsonDocument<256> doc;

  doc["topic"]     = "device_status";
  doc["deviceId"]  = DEVICE_ID;
  doc["status"]    = "online";
  doc["reason"]    = "device_connect";
  doc["timestamp"] = getTimestamp();

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] DEVICE STATUS");
  printPayload(payload);

  sendLoRa(payload);
}

void sendDeviceOffline()
{
  StaticJsonDocument<256> doc;

  doc["topic"]     = "device_status";
  doc["deviceId"]  = DEVICE_ID;
  doc["status"]    = "offline";
  doc["reason"]    = "device_disconnect";
  doc["timestamp"] = getTimestamp();

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] DEVICE OFFLINE");
  printPayload(payload);

  sendLoRa(payload);
}

void sendLogin(String rfid)
{
  StaticJsonDocument<256> doc;

  doc["topic"]     = "welderlogin";
  doc["deviceId"]  = DEVICE_ID;
  doc["rfid"]      = rfid;
  doc["timestamp"] = getTimestamp();

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] RFID LOGIN");
  printPayload(payload);

  sendLoRa(payload);
}

void sendLogout(String rfid)
{
  StaticJsonDocument<256> doc;

  doc["topic"]     = "welderlogout";
  doc["deviceId"]  = DEVICE_ID;
  doc["rfid"]      = rfid;
  doc["timestamp"] = getTimestamp();

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] RFID LOGOUT");
  printPayload(payload);

  sendLoRa(payload);
}

void sendWeldStart()
{
  StaticJsonDocument<384> doc;

  doc["topic"]           = "welder_start";
  doc["sessionId"]       = sessionID;
  doc["deviceId"]        = String(DEVICE_ID);
  doc["time"]            = getTimestamp();
  doc["organization_id"] = ORGANIZATION_ID;
  // Numeric rfid row id acked back by the server after welderlogin (see
  // radio_sx1262.cpp's handleDownlink(), cmd 0x02) -- NOT the raw card
  // UID string in currentRFID. Stays 0 if welding starts before the ack
  // arrives.
  doc["rfid"]            = currentRfidId;

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] WELDER START");
  printPayload(payload);

  sendLoRa(payload);
}

void sendWeldData()
{
  StaticJsonDocument<384> doc;

  doc["topic"]     = "welder_data";
  doc["sessionId"] = sessionID;
  doc["deviceId"]  = DEVICE_ID;
  doc["current"]   = Cal_Current;
  doc["voltage"]   = Cal_Voltage;

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] WELDER DATA");
  printPayload(payload);

  sendLoRa(payload);
}

void sendWeldStop()
{
  StaticJsonDocument<256> doc;

  doc["topic"]     = "welder_stop";
  doc["sessionId"] = sessionID;
  doc["stop_time"] = getTimestamp();
  doc["deviceId"]  = String(DEVICE_ID);

  String payload;
  serializeJson(doc, payload);

  Serial.println("[PAYLOAD] WELDER STOP");
  printPayload(payload);

  sendLoRa(payload);
}
