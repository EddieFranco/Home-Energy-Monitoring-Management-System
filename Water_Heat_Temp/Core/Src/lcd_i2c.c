#include "lcd_i2c.h"
#include <string.h>
#include <stdio.h>

// ===== Bit masks for both layouts =====

 //We define bit masks so our code can set/clear the right lines
  //without caring which physical P0..P7 they are on.

#if (LCD_PIN_LAYOUT == 'A')
// P0=D4, P1=D5, P2=D6, P3=D7, P4=BL, P5=EN, P6=RW, P7=RS
#define PIN_D4   0x01 // P0
#define PIN_D5   0x02 // P1
#define PIN_D6   0x04  // P2
#define PIN_D7   0x08 // P3
#define PIN_BL   0x10  // P4 (BackLight)
#define PIN_EN   0x20 // P5 (Enable)
#define PIN_RW   0x40 // P6 (Read/Write) - we keep this LOW (write)
#define PIN_RS   0x80 // P7 (Register Select) 0=command, 1=data
#elif (LCD_PIN_LAYOUT == 'B')
// P4=D4, P5=D5, P6=D6, P7=D7, P3=BL, P2=EN, P1=RW, P0=RS
#define PIN_D4   0x10  // P4
#define PIN_D5   0x20  // P5
#define PIN_D6   0x40 // P6
#define PIN_D7   0x80  // P7
#define PIN_BL   0x08 // P3
#define PIN_EN   0x04 // P2
#define PIN_RW   0x02  // P1
#define PIN_RS   0x01 // P0
#else
#error "LCD_PIN_LAYOUT must be 'A' or 'B'"
#endif

// This byte "remembers" the last control state we want (backlight on/off, RS/RW/EN off by default).
// We combine this with the data bits when we send to the PCF8574.
static uint8_t lcd_ctrl = 0; // backlight + RS/RW/EN zeros by default


/*
  i2c_write(byte):
  Sends ONE BYTE to the PCF8574 over I2C.
  The bits in 'byte' directly control the PCF8574 outputs (P0..P7).
*/
static inline void i2c_write(uint8_t data) {
  HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR, &data, 1, HAL_MAX_DELAY);
}


/*
  short_delay():
  Very short wait (~1–2 microseconds depending on optimization).
  Used around the EN pulse so the LCD can reliably latch data.
*/
static inline void short_delay(void) {
  // ~1–2 us at typical optimization levels
  for (volatile int i = 0; i < 200; i++) __NOP();
}


/*
  lcd_pulse_enable(data):
  Toggles the EN (enable) line HIGH then LOW, with a tiny delay.
  The LCD "reads" whatever D4..D7, RS/RW are at the rising edge of EN.
*/
static void lcd_pulse_enable(uint8_t data) {
  i2c_write(data | PIN_EN);
  short_delay();
  i2c_write(data & (uint8_t)~PIN_EN);
  short_delay();
}


/*
  lcd_write4bits(nibble, rs):
  Sends 4 bits (a nibble) to D4..D7 and generates the EN pulse.
  - nibble: lower 4 bits contain the data to put on D4..D7
  - rs: 0 = command; 1 = data/character
  RW is forced LOW (we only ever write to the LCD).
*/
static void lcd_write4bits(uint8_t nibble, uint8_t rs) {
	 // Start from current control state; keep backlight setting as-is.
  uint8_t data = lcd_ctrl & (PIN_BL);        // keep backlight state

  // Set RS depending on whether we're sending a command (0) or data (1).
  if (rs)
	  data |= PIN_RS;

  // Always force RW low for write mode (safe even if the board ties RW to GND).
  data &= (uint8_t)~PIN_RW;

  // Convert the low 4 bits of 'nibble' to the actual D4..D7 pins.
  uint8_t d = 0;
  if (nibble & 0x01) d |= PIN_D4;
  if (nibble & 0x02) d |= PIN_D5;
  if (nibble & 0x04) d |= PIN_D6;
  if (nibble & 0x08) d |= PIN_D7;
  // Put D4..D7 + control bits on the bus and pulse EN so the LCD latches it.
  lcd_pulse_enable((uint8_t)(data | d));
}


/*
  lcd_send(val, rs):
  Sends a FULL 8-bit value to the LCD by splitting it into two 4-bit chunks:
   - First the HIGH nibble (bits 7..4)
   - Then the LOW nibble  (bits 3..0)
  'rs' says if it's a command (0) or data (1).
*/
static void lcd_send(uint8_t val, uint8_t rs) {
  lcd_write4bits((uint8_t)((val >> 4) & 0x0F), rs);
  lcd_write4bits((uint8_t)(val & 0x0F), rs);
}

/*
  lcd_command(cmd):
  Convenience: send an 8-bit COMMAND (rs = 0).
  If the command is Clear (0x01) or Home (0x02), the LCD needs extra time,
  so we wait ~2 ms per the datasheet.
*/
static void lcd_command(uint8_t cmd) {
  lcd_send(cmd, 0);   // rs = 0 => command
  // Clear (0x01) and Home (0x02) need >1.5 ms
  if (cmd == 0x01 || cmd == 0x02)
	  HAL_Delay(2);  // wait >1.5 ms
}



/*
  LCD_SetBacklight(on):
  Turn the LCD backlight ON (1) or OFF (0).
  We update our control byte and immediately write it to the PCF8574 so the
  BL pin changes state.
*/
void LCD_SetBacklight(uint8_t on) {
  if (on)
	  lcd_ctrl |= PIN_BL;
  else
	  lcd_ctrl &= (uint8_t)~PIN_BL;

  i2c_write(lcd_ctrl); // update expander immediately
}


/*
  LCD_Init():
  Initialize the LCD after power-up.
  Steps:
    1) Wait ~50 ms so the LCD is ready.
    2) Turn backlight ON (nice to see it's alive).
    3) Special "4-bit mode" entry sequence (send 0x03 a few times, then 0x02).
    4) Function set: 4-bit, 2 line, 5x8 font (0x28).
    5) Display off (0x08), Clear (0x01), Entry mode increment (0x06),
       Display on, cursor off, blink off (0x0C).
*/
void LCD_Init(void) {
  HAL_Delay(50);               // power-up time

  LCD_SetBacklight(1);         // turn BL on early

  // 4-bit init sequence (as per HD44780 spec)
  lcd_write4bits(0x03, 0); HAL_Delay(5);
  lcd_write4bits(0x03, 0); HAL_Delay(5);
  lcd_write4bits(0x03, 0); HAL_Delay(1);
  lcd_write4bits(0x02, 0); // set 4-bit mode

  // Function set: 4-bit, 2-line (OK for 20x4), 5x8 font
  lcd_command(0x28);
  // Display off
  lcd_command(0x08);
  // Clear
  lcd_command(0x01); HAL_Delay(2);
  // Entry mode: increment cursor, no display shift
  lcd_command(0x06);
  // Display on, cursor off, blink off
  lcd_command(0x0C);
}

/*
  LCD_Clear():
  Clear the whole screen and move cursor to (0,0).
  (Internally calls lcd_command(0x01) which already waits long enough.)
*/
void LCD_Clear(void) {
	lcd_command(0x01);
}

/*
  LCD_Home():
  Move cursor to (0,0) without clearing text on the screen.
*/
void LCD_Home(void) {
	lcd_command(0x02);
}


/*
  Row base addresses for a 20x4 HD44780:
    Row 0 -> 0x00
    Row 1 -> 0x40
    Row 2 -> 0x14
    Row 3 -> 0x54
  (That's how 20x4 DDRAM is arranged.)
*/
// 20x4 row DDRAM bases
static const uint8_t row_addr[4] = {0x00, 0x40, 0x14, 0x54};



/*
  LCD_SetCursor(col, row):
  Move the cursor to a specific column and row (both 0-based).
  We compute the DDRAM address = row_base + col, then send command 0x80 | addr.
*/

void LCD_SetCursor(uint8_t col, uint8_t row) {
  if (row > 3)
	  row = 3;
  lcd_command((uint8_t)(0x80 | (row_addr[row] + col)));
}


/*
  LCD_Print(s):
  Print a C string starting at the current cursor position.
  For each character, send it as DATA (rs = 1) so it appears on the screen.
*/
void LCD_Print(const char *s) {
  while (*s) {
    lcd_send((uint8_t)*s++, 1);
  }
}


/*
  LCD_PrintAt(col, row, s):
  Shortcut: set the cursor position first, then print the string.
*/
void LCD_PrintAt(uint8_t col, uint8_t row, const char *s) {
  LCD_SetCursor(col, row);
  LCD_Print(s);
}


/*
  LCD_SelfTest():
  Quick test that writes 4 lines:
    Line 0: "I2C LCD OK"
    Line 1: "Addr:0x??"  (shows your 7-bit I2C address macro)
    Line 2: "Layout:A" or "Layout:B"
    Line 3: "20x4 Ready"
  Use this right after LCD_Init() to confirm wiring/address/layout.
*/
void LCD_SelfTest(void) {
  LCD_Clear();
  LCD_PrintAt(0,0, "I2C LCD OK");
  LCD_PrintAt(0,1, "Addr:");
  char buf[8];
  snprintf(buf, sizeof(buf), "0x%02X", LCD_I2C_ADDR_7B);
  LCD_Print(buf);
  LCD_PrintAt(0,2, "Layout:");

#if (LCD_PIN_LAYOUT == 'A')
  LCD_Print("A");
#else
  LCD_Print("B");
#endif
  LCD_PrintAt(0,3, "20x4 Ready");
}

// Optional: simple address scanner (writes via ITM if enabled)
/*
  I2C1_Scan_And_Log():
  Simple I2C scanner: probes addresses 0x03..0x77 and checks which respond.
  If you have printf via SWO/ITM/UART, you can print the hits to your console
  (the printf is commented out to avoid build issues if you don't have it set up).
  This is useful to find out if your backpack is 0x27 or 0x3F.
*/

void I2C1_Scan_And_Log(void) {
  for (uint8_t a = 0x03; a <= 0x77; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 5) == HAL_OK) {
      // If you have SWO/ITM printf, uncomment:
       printf("Found I2C device at 0x%02X\r\n", a);
      (void)a; // suppress unused if printf not used
    }
  }
}
