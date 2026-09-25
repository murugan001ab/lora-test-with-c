// include/sensors.h
//
// Voltage/current/feed-rate/gas-flow sampling: two analog ESP32 pins plus
// current on the ADS1115. Averages 200 samples per call, converts through
// the calibration factors in config.h, and pushes results to the DWIN
// display and the shared state in state.h.

#pragma once
#include <Arduino.h>

// Starts the ADS1115 and configures the analog sensor pins.
// Call once from setup(), after Wire.begin().
void initSensors();

// Blocking (~200 * ~200us per channel): samples all four channels,
// updates Cal_Voltage/Cal_Current/Cal_FeedRate/Cal_GasFlow in state.h,
// writes them to the DWIN display, and prints them to Serial.
void readSensors();

void printSensorReadings();
