#ifndef LCD_I2C_H_
#define LCD_I2C_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// This handle is created by CubeMX in the project (MX_I2C1_Init()).
// We declare it here so the LCD driver can use I2C1 without redefining it.
extern I2C_HandleTypeDef hi2c1;         // from CubeMX

// Try 0x27 first. If not found by I2C scan, try 0x3F.
#ifndef LCD_I2C_ADDR_7B
#define LCD_I2C_ADDR_7B  0x27           // 7-bit address (no shift)
#endif
#define LCD_I2C_ADDR     (LCD_I2C_ADDR_7B << 1) // HAL wants 8-bit

// ===== Choose one of these two popular backpack mappings =====
// Different PCF8574 backpacks map pins to the LCD differently.
// Layout A (default): P0..P3=D4..D7, P4=Backlight, P5=EN, P6=RW, P7=RS
// Layout B:           P4..P7=D4..D7, P3=Backlight, P2=EN, P1=RW, P0=RS
// If the screen shows junk or nothing (after adjusting contrast), flip this to 'B' and rebuild.
#ifndef LCD_PIN_LAYOUT
#define LCD_PIN_LAYOUT  'B'               // 'A' or 'B'
#endif

// Initialize the LCD in 4-bit mode over I2C.
// - Sets function (2 lines, 5x8 dots), entry mode, display on, clears screen.
// - Must be called after MX_I2C1_Init().
void LCD_Init(void);

// Clear the entire display and move the cursor to (0,0).
// Note: the HD44780 "clear" command takes ~1.6 ms; the driver already waits.
void LCD_Clear(void);

// Move the cursor to the home position (0,0) without clearing text.
// Also takes ~1.6 ms; the driver already waits.
void LCD_Home(void);

// Turn the LCD backlight on (1) or off (0).
// Backlight control works only if your backpack wires BL to the expander.
void LCD_SetBacklight(uint8_t on);

// Set the cursor to a specific column and row (0-based).
// - col: 0..19 for a 20x4
// - row: 0..3  for a 20x4
void LCD_SetCursor(uint8_t col, uint8_t row);  // 0-based col/row

// Print a null-terminated C string starting at the current cursor position.
// The cursor advances as characters are written.
void LCD_Print(const char *s);

// Convenience: set cursor then print a string in one call.
void LCD_PrintAt(uint8_t col, uint8_t row, const char *s);

// Optional helpers
// Quick self-test: writes 4 lines (title, I2C address, layout, "Ready").
// Use right after LCD_Init() to confirm wiring/address without your app logic.
void LCD_SelfTest(void);   // writes to all 4 lines

// I2C address scanner for I2C1 (0x03..0x77).
// If you have printf via SWO/ITM/UART enabled, it can log hits.
// Useful to discover whether your backpack is at 0x27 or 0x3F.
void I2C1_Scan_And_Log(void); // scans 0x03..0x77 and prints via ITM/semihosting if enabled

#ifdef __cplusplus
}
#endif

#endif /* LCD_I2C_H_ */

