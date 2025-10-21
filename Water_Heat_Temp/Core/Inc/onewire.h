#ifndef ONEWIRE_H
#define ONEWIRE_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

// ======= USER CONFIG =======
#define ONEWIRE_PORT      GPIOD       // Using Port D
#define ONEWIRE_PIN       GPIO_PIN_7	// Pin 7 on Port D
// ===========================

void ONEWIRE_Init(void);           // sets pin as open-drain with pull-up, starts DWT timer
bool ONEWIRE_Reset(void);          // returns true if a device pulls presence pulse

// Write and read single bits on the OneWire bus
void ONEWIRE_WriteBit(uint8_t b);
uint8_t ONEWIRE_ReadBit(void);

// Write and read whole bytes (8 bits) on the OneWire bus
void ONEWIRE_WriteByte(uint8_t byte);
uint8_t ONEWIRE_ReadByte(void);

// Timing-safe microsecond delay
void DWT_Delay_Init(void);
void DWT_Delay_us(uint32_t us);

#endif
