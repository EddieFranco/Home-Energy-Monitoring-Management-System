#include "telemetry.h"
#include <stdio.h>
#include <string.h>

#include "main.h"              // project-wide defs
#include "stm32f4xx_hal.h"     // UART_HandleTypeDef, HAL_UART_Transmit
#include "cmsis_os2.h"         // CMSIS-RTOS2 API

// -----------------------------------------------------------------------------
// Externals provided elsewhere in your project
// -----------------------------------------------------------------------------
extern UART_HandleTypeDef huart2;       // <- change to huart3, etc., if needed
extern osMutexId_t        UART_MutexHandle;  // <- created in main.c

// -----------------------------------------------------------------------------
// Module state
// -----------------------------------------------------------------------------
TelemetrySnapshot_t g_tel = {0};        // current snapshot
osMutexId_t         Telemetry_MutexHandle; // protects g_tel

static uint8_t txBuf[64];

// -----------------------------------------------------------------------------
// Public API
// -----------------------------------------------------------------------------
void Telemetry_UpdateSetpoint(float setpt_c)
{
    // Non-blocking try to avoid ever stalling producer tasks
    if (Telemetry_MutexHandle && osMutexAcquire(Telemetry_MutexHandle, 0) == osOK)
    {
        g_tel.setpoint_c = setpt_c;
        g_tel.t_ms       = osKernelGetTickCount();
        osMutexRelease(Telemetry_MutexHandle);
    }
}

void Telemetry_UpdateTemp(float temp_c)
{
    if (Telemetry_MutexHandle && osMutexAcquire(Telemetry_MutexHandle, 0) == osOK)
    {
        g_tel.temp_c = temp_c;
        g_tel.t_ms   = osKernelGetTickCount();
        osMutexRelease(Telemetry_MutexHandle);
    }
}

void Telemetry_Task(void *argument)
{
    (void)argument;
    const uint32_t period_ms = 100;                 // 10 Hz
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        // Take a snapshot (short, bounded critical section)
        TelemetrySnapshot_t s = {0};
        if (Telemetry_MutexHandle && osMutexAcquire(Telemetry_MutexHandle, 5) == osOK)
        {
            s = g_tel;
            osMutexRelease(Telemetry_MutexHandle);
        }

        // Format CSV: t_ms,setpoint,temp\n
        // Example: 123456,50.00,48.62\n
        const int n = snprintf((char*)txBuf, sizeof(txBuf),
                               "%lu,%.2f,%.2f\n",
                               (unsigned long)s.t_ms,
                               s.setpoint_c, s.temp_c);

        if (n > 0) {
            // --- Guard UART with your global mutex to avoid interleaved prints ---
            if (UART_MutexHandle && osMutexAcquire(UART_MutexHandle, 5) == osOK)
            {
                // Short, blocking TX (bytes << buffer size)
                HAL_UART_Transmit(&huart2, txBuf, (uint16_t)n, 10);
                osMutexRelease(UART_MutexHandle);
            }
            // If UART_MutexHandle is NULL (not created yet), we silently skip TX
            // to avoid racing with other UART users during early boot.
        }

        osDelayUntil(next + period_ms);
        next += period_ms;
    }
}

void Telemetry_Init(void)
{
    // Create the snapshot mutex
    const osMutexAttr_t mtxAttr = { .name = "Telemetry_Mutex" };
    Telemetry_MutexHandle = osMutexNew(&mtxAttr);

    // Seed defaults
    if (Telemetry_MutexHandle) {
        osMutexAcquire(Telemetry_MutexHandle, osWaitForever);
        g_tel.t_ms       = osKernelGetTickCount();
        g_tel.setpoint_c = 0.0f;
        g_tel.temp_c     = 0.0f;
        osMutexRelease(Telemetry_MutexHandle);
    }

    // Start the telemetry thread (low priority, small stack)
    static const osThreadAttr_t TelemetryTask_attributes = {
        .name       = "TelemetryTask",
        .stack_size = 256 * 4,
        .priority   = (osPriority_t)osPriorityLow
    };
    (void)osThreadNew(Telemetry_Task, NULL, &TelemetryTask_attributes);
}
