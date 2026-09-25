#include "shutdown_button.h"
#include "config.h"
#include "state.h"
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

      Serial.println("[SYSTEM] SHUTDOWN BUTTON HELD - CLOSING OUT SESSION");

      // Close out whatever's open before going offline -- otherwise this
      // graceful path would leave the exact same dangling weld/login state
      // an abrupt power cut does, just with extra steps. Order matters:
      // weld first (it depends on a login being open), then the login,
      // then the device itself.
      if (weldingStarted)
      {
        Serial.println("[SYSTEM] Weld active - sending welder_stop");
        sendWeldStop();
        weldingStarted = false;
      }

      if (loggedIn)
      {
        Serial.println("[SYSTEM] Operator logged in - sending welderlogout");
        sendLogout(currentRFID);

        loggedIn      = false;
        currentRFID   = "";
        sessionID     = "";
        currentRfidId = 0;
        systemState   = IDLE;
      }

      Serial.println("[SYSTEM] SENDING OFFLINE STATUS");
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
