#  Water Heater Temperature Control System  
### Using STM32F407, FreeRTOS, PID Control, and ATM90E32AS Energy Meter

---

## Overview

This project implements a **closed-loop water heater temperature control system** using a **PID controller** under **FreeRTOS** on an **STM32F407G-DISC1** microcontroller, with a **BTS7960 PWM driver** for high-current heater control and an **ATM90E32AS poly-phase energy metering IC** (SPI) for real-time voltage, current, and power monitoring.


It uses:
- **STM32F407G-DISC1 microcontroller** as the main processing unit running FreeRTOS and coordinating all system tasks  
- **DS18B20 temperature sensor** for precise water temperature feedback  
- **BTS7960 43A motor driver** for high-current PWM heater control  
- **ATM90E32AS poly-phase energy metering IC** for accurate voltage, current, and power measurement via SPI  
- **I²C 16×2 LCD display** for real-time system information such as temperature, power, and control mode  
- **Python Matplotlib** for real-time data visualization over UART on a host PC  


The goal is to maintain stable heater performance, energy efficiency, and safety through modular firmware and real-time monitoring.

---

## System Images

| Description | Image |
|--------------|--------|
| PCB Atmel M90E32AS Power Monitor Schematic Overview | ![Schematic Overview](Images/M90E32AS_schematic.JPG) |
| PCB Atmel M90E32AS Power Monitor Layout | ![PCB Layout](Images/M90E32AS_layout.JPG) |
| PCB Atmel M90E32AS Power Monitor 3D | ![PCB 3D](Images/M90E32AS_3D.JPG) |
| Water Heater Temperature Control Hardware Prototype Setup | ![System Setup](Images/system_setup.jpg) |
| Water Heater Temperature Control Live Data Visualization | ![Python Plot](Images/python_plot.png) |
| Water Heater Temperature Control Optional System Block Diagram | ![Block Diagram](Images/system_diagram.png) |


---

## ⚙️ Features

- PID-based temperature control (tunable Kp, Ki, Kd)
- Modular **FreeRTOS** architecture with separate tasks for temperature, PID, UI, fault, and power monitoring
- Energy monitoring with **ATM90E32AS** over SPI
- Real-time UART telemetry for data logging and live plotting
- Calibration routines for voltage and current channels
- Safety layer for overcurrent and sensor fault handling
- Expandable Python GUI for monitoring and analysis

---

## System Architecture

---

##  Hardware Setup

| Component | Description |
|------------|--------------|
| MCU | STM32F407G-DISC1 |
| Driver | BTS7960 43A Dual H-Bridge Motor Driver |
| Sensor | DS18B20 temperature sensor |
| Energy Meter | ATM90E32AS (poly-phase metering IC, SPI3) |
| Power Supply | LM2596 buck + Mean Well LRS-350-12 |
| Display | 16×2 LCD via I²C (optional) |
| Load | 12V resistive water heater element (~300W) |

**Connections**
- **PWM (Heater Control / BTS7960 Driver):**  
  - TIM3_CH3 → PB0 (LPWM)  
  - TIM3_CH4 → PB1 (RPWM)  
  - VCC (BTS7960 logic) → 5V  
  - GND → Common ground with STM32  
  - Motor output terminals → Heater load  

- **SPI3 (Energy Meter / ATM90E32AS):**  
  - PC10 → SCK  
  - PC11 → MISO  
  - PC12 → MOSI  
  - PC9  → CS (Chip Select)  

- **Temperature Sensor (DS18B20):**  
  - 1-Wire Data → e.g., PA8 (configurable GPIO input/output)  
  - 4.7 kΩ pull-up to 3.3 V  

- **LCD Display (I²C 16×2):**  
  - PB6 → SCL  
  - PB7 → SDA  
  - 5V and GND shared with STM32 board  

- **UART (Telemetry / Python Visualization):**  
  - USART2_TX → PA2  
  - USART2_RX → PA3  
  - Baud rate: 115200 bps, 8N1 configuration  

- **Power Supply:**  
  - 12 V DC input → LM2596 buck → 5 V and 3.3 V rails for logic  
  - Common GND between STM32, BTS7960, and ATM90E32AS
 


---

##  Firmware Modules

- **PWM / BTS7960 Driver**
  - Headers: `Core/Inc/bts7960_pwm.h`
  - Sources: `Core/Src/bts7960_pwm.c`

- **Energy Meter / ATM90E32AS**
  - Headers: `Core/Inc/atm90e32.h`
  - Sources: `Core/Src/atm90e32.c`

- **Temperature Sensor / DS18B20**
  - Headers: `Core/Inc/ds18b20.h`
  - Sources: `Core/Src/ds18b20.c`

- **LCD (I²C 16×2, PCF8574 Backpack)**
  - Headers: `Core/Inc/lcd_i2c.h`
  - Sources: `Core/Src/lcd_i2c.c`

- **PID Controller / Control Task**
  - Headers: `Core/Inc/pid_task.h`
  - Sources: `Core/Src/pid_task.c`

- **UART Telemetry / Serial Communication**
  - Headers: `Core/Inc/telemetry.h`
  - Sources: `Core/Src/telemetry.c`


### 🔹 RTOS Tasks
| Task | Function |
|------|-----------|
| `TempTask` | Reads DS18B20 temperature and queues it |
| `PIDTask` | Computes control output and sets PWM |
| `SafetyTask` | Handles overcurrent and fault events |
| `UITask` | Updates LCD or UART display |
| `PowerTask` | Reads ATM90E32AS power/voltage data |
---
## 🧵 RTOS Architecture (Mermaid)

The diagram below shows the communication between tasks, queues, semaphores, and mutexes in the system.  

```mermaid
---
config:
  theme: dark
  look: classic
  layout: fixed
---
flowchart TB
 subgraph Legend["Legend"]
    direction LR
        L1["Semaphores"]
        L2["Mutex"]
        L3["Task"]
        L4["Queue"]
  end
 subgraph LCD["LCD 16×2( LCD Task)"]
        L5["Line 1:  P:###.#W   T:##.#°C"]
        L6["Line 2:  SP:##.#°C  MODE:XXXX"]
  end
    TIM_ISR["Timer ISR (100 ms)"] -- give --> PID_TickSem["PID_TickSem (BinSem)"]
    ADC_ISR["POWER SENSOR ISR"] -- give --> Power_DoneSem["PowerDoneSem (BinSem)"]
    BTN_ISR["Button EXTI ISR"] -- give --> BtnEventSem["BtnEventSem (BinSem)"]
    TEMP["Temp Sensor Task (P3)"] -- send temp --> TempQ["TempQ"]
    TempQ --> PID["PID Control Task (P4)"] & UI["UI/LCD Task (P2)"]
    UI -- send setpoint --> SetpointQ["SetpointQ"]
    SetpointQ --> PID
    POWER["Power Monitor Task (P1)"] -- send power --> PowerQ["PowerQ"]
    PowerQ --> SAFETY["Safety/Fault Task (P5)"] & UI
    POWER -- send alert --> FaultQ["FaultQ"]
    FaultQ --> SAFETY
    PID_TickSem --> PID
    Power_DoneSem --> POWER
    BtnEventSem --> UI
    UI -- take/give --> LCD_Mutex["LCD_Mutex"]
    TEMP -- take/give --> OneWire_Mutex["1-Wire_Mutex"]
    PID <-- take/give --> PIDParam_Mutex["PIDParam_Mutex"]
    UI -- edit params --> PIDParam_Mutex
    ANY["Any task"] -- printf --> UART_Mutex["UART_Mutex"]
    UI -- write lines --> LCD
    LCD_Mutex <--> SAFETY
    title["Water Heater RTOS Flowchart"]
     L1:::Semaphores
     L2:::Mutex
     L3:::Task
     L4:::Queue
     PID_TickSem:::Semaphores
     Power_DoneSem:::Semaphores
     BtnEventSem:::Semaphores
     TEMP:::Task
     TempQ:::Queue
     PID:::Task
     UI:::Task
     SetpointQ:::Queue
     POWER:::Task
     PowerQ:::Queue
     SAFETY:::Task
     FaultQ:::Queue
     LCD_Mutex:::Mutex
     OneWire_Mutex:::Mutex
     PIDParam_Mutex:::Mutex
     UART_Mutex:::Mutex
    classDef Semaphores fill:#ccccff,stroke:#0000ff,stroke-width:2px,color:#000000
    classDef Mutex fill:#ffcccc,stroke:#ff0000,stroke-width:2px,color:#000000
    classDef Task fill:#00ddff,stroke:#0000aa,stroke-width:2px,color:#000000
    classDef Queue fill:#00ff99,stroke:#00aa00,stroke-width:2px,color:#000000
    style L5 fill:#D50000
    style L6 fill:#D50000
    style LCD fill:#2962FF
    style title fill:none,stroke:none,fontSize:24px,fontWeight:bold,color:#00C853

```
---
## 📊 Python Visualization

A Python script streams UART data from the STM32 for live visualization.

Example line format:

