#include "onewire.h"

// Pull the bus line LOW (drive 0)
static inline void ow_low(void) {
  HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_RESET);
}

// Release the bus line (OD high = Hi-Z, pulled up by resistor)
static inline void ow_release(void) {
  HAL_GPIO_WritePin(ONEWIRE_PORT, ONEWIRE_PIN, GPIO_PIN_SET); // OD high = released
}

// Read the current level of the bus line (HIGH or LOW)
static inline GPIO_PinState ow_read(void) {
  return HAL_GPIO_ReadPin(ONEWIRE_PORT, ONEWIRE_PIN);
}

// Initialize the DWT cycle counter for microsecond delays
// DWT delay (as you already had)
void DWT_Delay_Init(void) {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// Delay for a given number of microseconds using DWT
void DWT_Delay_us(uint32_t us) {
  uint32_t cycles = (SystemCoreClock/1000000U) * us;
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
}

// Initialize the OneWire pin: Open-Drain with Pull-Up
void ONEWIRE_Init(void) {
  // Enable port clock for PD (or your chosen port)
  //__HAL_RCC_GPIOD_CLK_ENABLE();  // <-- adjust if you moved pins
  DWT_Delay_Init();

  GPIO_InitTypeDef i = {0};
  i.Pin = ONEWIRE_PIN;
  i.Mode = GPIO_MODE_OUTPUT_OD;          // always OD output
  i.Pull = GPIO_PULLUP;                  // keep internal PU; still need 4.7k external
  i.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(ONEWIRE_PORT, &i);

  ow_release(); // idle high
}



// Send reset pulse and check for presence pulse
// Returns true if device responds
bool ONEWIRE_Reset(void) {
  ow_low();
  DWT_Delay_us(500);         // 480–960 us low
  ow_release();
  DWT_Delay_us(70);          // wait before sampling presence
  bool present = (ow_read() == GPIO_PIN_RESET);
  DWT_Delay_us(430);         // finish the slot
  return present;
}


// Write a single bit to the bus
// '1' = short low, then release
// '0' = long low
void ONEWIRE_WriteBit(uint8_t b) {
  if (b) {
    // Write '1': low ≥1us, sample window 15us, release until end (~60us total)
    ow_low();
    DWT_Delay_us(3);        // short low for '1'
    ow_release();
    DWT_Delay_us(60);
  } else {
    // Write '0': low 60us
    ow_low();
    DWT_Delay_us(60);		 // long low for '0'
    ow_release();
    DWT_Delay_us(5);
  }
}

// Read a single bit from the bus
uint8_t ONEWIRE_ReadBit(void) {
  uint8_t bit;
  ow_low();
  DWT_Delay_us(3);           // start read slot
  ow_release();
  DWT_Delay_us(12);          // sample around 12–15us from slot start
  bit = (ow_read() == GPIO_PIN_SET);
  DWT_Delay_us(50);          // finish ~60–65us total
  return bit;
}

// Write a full byte (8 bits), LSB first
void ONEWIRE_WriteByte(uint8_t byte) {
  for (int i = 0; i < 8; i++) {
    ONEWIRE_WriteBit(byte & 0x01);
    byte >>= 1;
  }
}


// Read a full byte (8 bits), LSB first
uint8_t ONEWIRE_ReadByte(void) {
  uint8_t val = 0;
  for (int i = 0; i < 8; i++) {
    if (ONEWIRE_ReadBit())
    	val |= (1U << i);   // LSB first
  }
  return val;
}
