#pragma once
#include "cmsis_os2.h"
#include <stdint.h>

// Minimal snapshot: time (ms), setpoint (°C), temperature (°C)
typedef struct {
    uint32_t t_ms;
    float    setpoint_c;
    float    temp_c;
} TelemetrySnapshot_t;

#ifdef __cplusplus
extern "C" {
#endif

extern osMutexId_t Telemetry_MutexHandle;   // created in Telemetry_Init()
void Telemetry_Init(void);                  // create mutex + start task
void Telemetry_UpdateSetpoint(float setpt_c);
void Telemetry_UpdateTemp(float temp_c);
void Telemetry_Task(void *argument);        // RTOS thread (10 Hz)

#ifdef __cplusplus
}
#endif
