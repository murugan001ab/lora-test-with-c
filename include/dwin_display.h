// include/dwin_display.h
//
// DWIN HMI display, driven over UART2 with the 5A A5 VP-write protocol.

#pragma once
#include <Arduino.h>

// Starts DWINSerial (UART2) and resets the display to page 0.
// Call once from setup().
void initDwinDisplay();

// Raw VP write: 5A A5 05 82 <addr_hi><addr_lo> <val_hi><val_lo>
void DWIN_Write(uint16_t address, uint16_t value);

// Convenience wrapper for the page-change VP (0x0084).
void DWIN_ChangePage(uint8_t page);
