#include "shutdown_button.h"
#include "config.h"
#include "payloads.h"

#include <esp_sleep.h>

static unsigned long shutdownPressStart = 0;
static bool shutdownTriggered = false;

void initShutdownButton()
{
  pinMode(SHUTDOWN_BTN_PIN, INPUT_PULLUP);
}

void checkShutdownButton()
{
  if (digitalRead(SHUTDOWN_BTN_PIN) == LOW)
  {
    if (shutdownPressStart == 0)
    {
      shutdownPressStart = millis();
    }
    else if (!shutdownTriggered && millis() - shutdownPressStart >= SHUTDOWN_HOLD_MS)
    {
      shutdownTriggered = true;

      Serial.println("[SYSTEM] SHUTDOWN BUTTON HELD - SENDING OFFLINE STATUS");
      sendDeviceOffline();

      Serial.println("[SYSTEM] Entering deep sleep");
      delay(200);

      esp_deep_sleep_start();
    }
  }
  else
  {
    shutdownPressStart = 0;
    shutdownTriggered  = false;
  }
}
