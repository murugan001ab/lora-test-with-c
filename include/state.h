// include/state.h
//
// Shared runtime state: login/session/welding status and the calibrated
// sensor values, plus the device clock base used by time_utils. Declared
// here as externs; defined once in state.cpp. Any module that reads or
// updates session/weld/clock state includes this.

#pragma once
#include <Arduino.h>

enum SystemState
{
  IDLE,
  LOGIN_PENDING,
  LOGGED_IN,
  LOGOUT_PENDING
};

extern SystemState systemState;

extern bool loggedIn;
extern bool weldingStarted;

extern String currentRFID;
extern String sessionID;

extern unsigned long loginDebounceTime;
extern unsigned long lastWeldDataTime;

// Calibrated sensor readings — written by sensors.cpp, read by payloads.cpp
// and main.cpp's weld-state machine.
extern float Cal_Voltage;
extern float Cal_Current;
extern float Cal_FeedRate;
extern float Cal_GasFlow;

// Device clock. epochBase is set to 0 at boot and updated by setEpoch()
// (time_utils.h) whenever a TIME_SYNC downlink arrives.
extern unsigned long baseMillis;
extern unsigned long epochBase;

// Numeric rfid row id (rfids.id) the server acks back after welderlogin
// (see radio_sx1262.cpp's handleDownlink(), cmd 0x02 RFID_ACK). 0 means
// "not yet acked" -- reset on every fresh login so a stale id from a
// previous session is never sent. welder_start echoes this (not
// currentRFID) back to the server.
extern uint32_t currentRfidId;
