// include/time_utils.h
//
// Device clock derived from millis() + a synced Unix epoch base.

#pragma once
#include <Arduino.h>

// Rebase the clock: baseMillis = millis(), epochBase = epochSeconds.
// Call once at boot with 0, and again whenever a TIME_SYNC downlink
// arrives (see radio_sx1262.cpp).
void setEpoch(uint32_t epochSeconds);

// ISO-8601 UTC timestamp ("YYYY-MM-DDTHH:MM:SSZ") derived from the
// current epoch base + elapsed millis(). Reads as 1970 epoch time until
// the first TIME_SYNC downlink is received.
String getTimestamp();

// Random v4-style UUID, used for welding session IDs.
String generateUUID();
