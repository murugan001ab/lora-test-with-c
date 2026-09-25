#include "time_utils.h"
#include "state.h"
#include <esp_system.h> // esp_random()

void setEpoch(uint32_t epochSeconds)
{
  baseMillis = millis();
  epochBase  = epochSeconds;
}

String getTimestamp()
{
  unsigned long elapsedSeconds = (millis() - baseMillis) / 1000UL;
  unsigned long currentEpoch   = epochBase + elapsedSeconds;

  unsigned long days         = currentEpoch / 86400UL;
  unsigned long secondsOfDay = currentEpoch % 86400UL;

  int hour   = secondsOfDay / 3600;
  int minute = (secondsOfDay % 3600) / 60;
  int second = secondsOfDay % 60;

  int year = 1970;
  unsigned long remainingDays = days;

  while (true)
  {
    bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
    int daysInYear = leap ? 366 : 365;

    if (remainingDays >= (unsigned long)daysInYear)
    {
      remainingDays -= daysInYear;
      year++;
    }
    else
    {
      break;
    }
  }

  int monthDays[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

  bool leap = ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0));
  if (leap)
  {
    monthDays[1] = 29;
  }

  int month = 0;
  while (remainingDays >= (unsigned long)monthDays[month])
  {
    remainingDays -= monthDays[month];
    month++;
  }
  month++;

  char buffer[40];
  sprintf(
    buffer,
    "%04d-%02d-%02dT%02d:%02d:%02dZ",
    year, month, (int)remainingDays + 1,
    hour, minute, second
  );

  return String(buffer);
}

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
