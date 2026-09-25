#include "dwin_display.h"
#include "config.h"

HardwareSerial DWINSerial(2);

void initDwinDisplay()
{
  DWINSerial.begin(115200, SERIAL_8N1, DWIN_RX_PIN, DWIN_TX_PIN);
  delay(200);

  DWIN_ChangePage(0);

  Serial.println("[DWIN] UART2 initialized");
}

void DWIN_Write(uint16_t address, uint16_t value)
{
  uint8_t frame[8];

  frame[0] = 0x5A;
  frame[1] = 0xA5;
  frame[2] = 0x05;
  frame[3] = 0x82;
  frame[4] = (address >> 8) & 0xFF;
  frame[5] = address & 0xFF;
  frame[6] = (value >> 8) & 0xFF;
  frame[7] = value & 0xFF;

  DWINSerial.write(frame, 8);
}

void DWIN_ChangePage(uint8_t page)
{
  DWIN_Write(0x0084, page);
}
