#include <string.h>
#include <math.h>          // fabsf()
#include "ds18b20.h"
#include "onewire.h"       // ONEWIRE_Reset/WriteByte/ReadByte/ReadBit
// HAL_Delay() and HAL_GetTick() come from STM32 HAL (included elsewhere)

// ===== ROM / Function codes (DS18B20 datasheet) =====
#define CMD_READ_PWRSUP  0xB4  // Power supply test (parasitic or VDD)
#define CMD_SKIP_ROM     0xCC  // Single-drop bus: address "all" (just one)
#define CMD_CONVERT_T    0x44  // Start temperature conversion
#define CMD_READ_SCRATCH 0xBE  // Read 9-byte scratchpad

// ===== Internal state for the simple watchdog approach =====
static float    g_last_good_c     = 25.0f;  // last stable temperature
static uint32_t g_last_ok_ms      = 0;      // last time we saw a valid read
static bool     g_fault_latched   = false;  // latched fault flag

// ---------- Private CRC-8 (Dallas/Maxim) ----------
static uint8_t ds_crc8(const uint8_t *data, int len) {
  uint8_t crc = 0;
  for (int i = 0; i < len; i++) {
    uint8_t inbyte = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      uint8_t mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) crc ^= 0x8C; // 0x8C is reversed 0x31 (poly 0x31)
      inbyte >>= 1;
    }
  }
  return crc;
}

// ---------- Power mode check (parasitic vs VDD) ----------
static bool DS18B20_ReadPowerSupply(bool *is_parasitic) {
  if (!ONEWIRE_Reset()) return false;
  ONEWIRE_WriteByte(CMD_SKIP_ROM);
  ONEWIRE_WriteByte(CMD_READ_PWRSUP);
  // Device writes 0 if parasitic, 1 if externally powered
  uint8_t b = ONEWIRE_ReadBit();
  *is_parasitic = (b == 0);
  return true;
}

// ---------- Start conversion (single device on the bus) ----------
bool DS18B20_StartConversion_SkipROM(void) {
  if (!ONEWIRE_Reset()) return false;
  ONEWIRE_WriteByte(CMD_SKIP_ROM);
  ONEWIRE_WriteByte(CMD_CONVERT_T);
  return true;
}

// ---------- Read 9-byte scratchpad ----------
bool DS18B20_ReadScratchpad(uint8_t *buf9) {
  if (!ONEWIRE_Reset()) return false;
  ONEWIRE_WriteByte(CMD_SKIP_ROM);
  ONEWIRE_WriteByte(CMD_READ_SCRATCH);
  for (int i = 0; i < 9; i++) buf9[i] = ONEWIRE_ReadByte();
  return true;
}

// ---------- LOW-LEVEL one-shot read (blocking) ----------
// Returns true + *out_celsius on success. Handles:
// - conversion wait (parasitic or external power),
// - 9-byte scratchpad read,
// - CRC-8 check,
// - range check (+ rejects the infamous 85.0 °C "not converted yet" value).
static bool DS18B20_ReadRawC(float *out_celsius) {
  bool parasitic = false;
  if (!DS18B20_ReadPowerSupply(&parasitic)) return false;

  // 1) Kick conversion
  if (!ONEWIRE_Reset()) return false;
  ONEWIRE_WriteByte(CMD_SKIP_ROM);
  ONEWIRE_WriteByte(CMD_CONVERT_T);

  // 2) Wait for conversion to finish
  if (parasitic) {
    // In parasitic mode the MCU must supply strong pull-up during conversion.
    // If you don't do that, use a conservative fixed delay:
    HAL_Delay(750);
  } else {
    // With VDD power, the device releases the bus when done (reads '1')
    uint32_t timeout_ms = 800;
    while (timeout_ms--) {
      if (ONEWIRE_ReadBit()) break; // done when line reads '1'
      HAL_Delay(1);
    }
    if ((int32_t)timeout_ms < 0) return false; // timed out
  }

  // 3) Read scratchpad (9 bytes)
  uint8_t s[9];
  if (!DS18B20_ReadScratchpad(s)) return false;

  // 4) CRC must match (Dallas CRC returns 0 for correct 9-byte buffer)
  if (ds_crc8(s, 9) != 0) return false;

  // 5) Convert raw temperature (LSB/MSB little-endian). Default 12-bit.
  int16_t raw = (int16_t)((s[1] << 8) | s[0]);
  float c = (float)raw / 16.0f;

  // 6) Reject the famous 85.0°C "power-up" value (means conversion likely failed)
  if (fabsf(c - 85.0f) < 0.01f) return false;

  // 7) Physical range for DS18B20
  if (c < -55.0f || c > 125.0f) return false;

  *out_celsius = c;
  return true;
}

// ---------- PUBLIC: convenience read (raw, blocking) ----------
bool DS18B20_ReadTempC(float *out_celsius) {
  return DS18B20_ReadRawC(out_celsius);
}

// ---------- PUBLIC: initialize watchdog state ----------
void DS18B20_Init(float seed_celsius) {
  g_last_good_c   = seed_celsius;       // start from sensible ambient
  g_last_ok_ms    = HAL_GetTick();      // "we just started; don't trip yet"
  g_fault_latched = false;              // no fault at boot
}

// ---------- PUBLIC: simple watchdog read ----------
// - Always writes *out_celsius the latest stable value (g_last_good_c).
// - Returns true when a *new* good sample was obtained this call.
// - If no good sample arrives for DS18B20_FAULT_TIMEOUT_MS, latch a fault
//   and set *fault_tripped_now=true on that transition.
bool DS18B20_ReadTempC_Watchdog(float *out_celsius, bool *fault_tripped_now) {
  if (fault_tripped_now) *fault_tripped_now = false;

  float raw = 0.0f;
  bool ok   = DS18B20_ReadRawC(&raw);

  if (ok) {
    // Update the stable value and "last success" time
    g_last_good_c = raw;
    g_last_ok_ms  = HAL_GetTick();
  }

  // Trip only if we've had NO good reads for a long time
  uint32_t now = HAL_GetTick();
  if (!g_fault_latched && g_last_ok_ms != 0 &&
      (now - g_last_ok_ms) > DS18B20_FAULT_TIMEOUT_MS) {
    g_fault_latched = true;
    if (fault_tripped_now) *fault_tripped_now = true;  // tell caller to trip Safety
  }

  if (out_celsius) *out_celsius = g_last_good_c;  // always hand out a good/stable number
  return ok;                                      // "ok" means we updated g_last_good_c
}

// ---------- PUBLIC: fault helpers ----------
bool DS18B20_FaultLatched(void) { return g_fault_latched; }

void DS18B20_FaultClear(void) {
  g_fault_latched = false;
  // Do NOT reset g_last_ok_ms here; it will update on the next success.
}
