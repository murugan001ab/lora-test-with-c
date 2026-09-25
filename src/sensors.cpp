#include "sensors.h"
#include "config.h"
#include "state.h"
#include "dwin_display.h"

#include <Wire.h>
#include <Adafruit_ADS1X15.h>

static Adafruit_ADS1115 ads;

static const float Cal_Factor  = ref_volt / 3.3f;
// Kept from the original firmware; not currently used by any conversion
// below, preserved in case a future sensor channel needs the 3.0V-ref
// scale instead of Cal_Factor's 3.3V-ref scale.
static const float Cal_Factor1 __attribute__((unused)) = ref_volt1 / 3.0f;

static int16_t rawADS = 0;

void initSensors()
{
  if (!ads.begin())
  {
    Serial.println("[ADS1115] ERROR - NOT FOUND");
  }
  else
  {
    Serial.println("[ADS1115] ADS1115 OK");
  }

  ads.setGain(GAIN_TWOTHIRDS);

  pinMode(Voltage_Sens, INPUT);
  pinMode(FeedRate_Sens, INPUT);
  pinMode(GasFlow_Sens, INPUT);
}

void readSensors()
{
  long voltageSum = 0;
  long feedSum    = 0;
  long gasSum     = 0;
  long currentSum = 0;

  const int sampleCount = 200;

  for (int i = 0; i < sampleCount; i++)
  {
    voltageSum += analogRead(Voltage_Sens);
    feedSum    += analogRead(FeedRate_Sens);
    gasSum     += analogRead(GasFlow_Sens);
    currentSum += ads.readADC_SingleEnded(0);

    delayMicroseconds(200);
  }

  float RawADCVoltage  = voltageSum / (float)sampleCount;
  float RawADCFeedRate = feedSum / (float)sampleCount;
  float RawADCGasFlow  = gasSum / (float)sampleCount;

  rawADS = (int16_t)(currentSum / sampleCount);

  Cal_Voltage  = RawADCVoltage  * Vg_Cal_Factor * Cal_Factor;
  Cal_Current  = rawADS * Ct_Cal_Factor;
  Cal_FeedRate = RawADCFeedRate * FR_Cal_Factor * Cal_Factor;
  Cal_GasFlow  = RawADCGasFlow  * GF_Cal_Factor * Cal_Factor;

  DWIN_Write(0x1220, (uint16_t)Cal_Voltage);
  DWIN_Write(0x1240, (uint16_t)Cal_Current);
  DWIN_Write(0x1260, (uint16_t)Cal_FeedRate);
  DWIN_Write(0x1280, (uint16_t)Cal_GasFlow);

  printSensorReadings();
}

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
