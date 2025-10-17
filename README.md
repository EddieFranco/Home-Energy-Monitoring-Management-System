# 🔥 Water Heater Temperature Control System  
### Using STM32F407, FreeRTOS, PID Control, and ATM90E32AS Energy Meter

---

## 🧠 Overview

This project implements a **closed-loop water heater temperature control system** using a **PID controller** under **FreeRTOS** on an **STM32F407G-DISC1** microcontroller.

It uses:
- **DS18B20 temperature sensor** for precise water temperature feedback  
- **BTS7960 43A motor driver** for high-current PWM heater control  
- **ATM90E32AS poly-phase energy metering IC** for accurate voltage, current, and power measurement via SPI  
- **Python Matplotlib** for real-time data visualization over UART  

The goal is to maintain stable heater performance, energy efficiency, and safety through modular firmware and real-time monitoring.

---

## 🖼️ System Images

| Description | Image |
|--------------|--------|
| PCB design or 3D render | ![PCB Design](images/pcb_design.png) |
| Hardware prototype setup | ![System Setup](images/system_setup.jpg) |
| Live data visualization | ![Python Plot](images/python_plot.png) |
| Optional system block diagram | ![Block Diagram](images/system_diagram.png) |

> 💡 *Replace these image paths with your actual image filenames once uploaded.*

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

## 🧩 System Architecture

