#ifndef DS18B20_H
#define DS18B20_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Simple DS18B20 driver with a time-based watchdog:
 * - Always returns the latest stable temperature (in *out_celsius).
 * - Marks a FAULT only if we get NO good reads for FAULT_TIMEOUT_MS.
 * - Beginner-friendly: minimal moving parts, strong comments.
 *
 * You still need ONEWIRE_* functions (in onewire.h/.c) and STM32 HAL
 * for HAL_Delay() and HAL_GetTick().
 */

// ------------ Public configuration ------------
#define DS18B20_FAULT_TIMEOUT_MS   15000U   // 15s without any good read => fault

// ------------ Public API ------------

// Initialize internal state (call once at startup).
// seed_celsius: a sensible starting value (e.g., 25.0f)
void DS18B20_Init(float seed_celsius);

// Read temperature with watchdog:
// - Returns true if a *new* valid reading was obtained this call,
//   false if not (but *out_celsius still returns the last stable value).
// - If fault_tripped_now != NULL, it is set to true only when the
//   function detects that the sensor has been bad for > timeout and
//   the fault just latched on THIS call.
bool DS18B20_ReadTempC_Watchdog(float *out_celsius, bool *fault_tripped_now);

// Is the driver currently latched in a sensor-fail state?
bool DS18B20_FaultLatched(void);

// Clear a latched fault (optional, if your Safety task supports clearing).
void DS18B20_FaultClear(void);

// ------------ Optional low-level helpers  ------------
bool DS18B20_StartConversion_SkipROM(void);  // single-drop bus
bool DS18B20_ReadScratchpad(uint8_t *buf9);  // 9 bytes
bool DS18B20_ReadTempC(float *out_celsius);  // convenience: one-shot blocking read

#endif
