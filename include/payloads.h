// include/payloads.h
//
// JSON payload builders for every uplink topic. Each function serializes
// its topic's fields (device.h/state.h) and sends the result via
// sendLoRa() (radio_sx1262.h).

#pragma once
#include <Arduino.h>

void printPayload(const String &payload);

void sendDeviceOnline();
void sendDeviceOffline();

void sendLogin(String rfid);
void sendLogout(String rfid);

void sendWeldStart();
void sendWeldData();
void sendWeldStop();
