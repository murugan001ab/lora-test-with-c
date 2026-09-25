#include "state.h"

SystemState systemState = IDLE;

bool loggedIn       = false;
bool weldingStarted = false;

String currentRFID = "";
String sessionID   = "";

unsigned long loginDebounceTime = 0;
unsigned long lastWeldDataTime  = 0;

float Cal_Voltage  = 0.0;
float Cal_Current  = 0.0;
float Cal_FeedRate = 0.0;
float Cal_GasFlow  = 0.0;

unsigned long baseMillis = 0;
unsigned long epochBase  = 0;

uint32_t currentRfidId = 0;
