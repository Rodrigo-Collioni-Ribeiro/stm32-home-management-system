# Embedded Home Management System — STM32F439

A real-time home automation controller built on the STM32F439 microcontroller, developed for RMIT's Embedded System Design and Implementation course.

**Team:** Rodrigo Collioni Ribeiro, Anuraj Verma

## Overview

This system simulates a home automation controller that monitors room temperature via a potentiometer, controls light/fan/heater/cooler outputs, accepts commands over serial, and reports status automatically. The design centers on a **non-blocking software scheduler** — a single hardware timer coordinating debounce, lockout, and transmission timing for every subsystem, avoiding both interrupt complexity and multiple hardware timers.

## Key Features

- **Software-scheduled multitasking** — TIM2 generates a 1ms tick; a single scheduler decrements independent software counters for debounce, lockout, and transmission timing across all peripherals, keeping the main loop fully non-blocking.
- **ADC-based temperature simulation** — ADC3 reads a potentiometer and linearly maps it to a −30°C to +55°C range, stored in centi-degrees to avoid floating-point math.
- **Serial command protocol over USART3** — Custom encoder/decoder at 57,600 bps (8 data bits, odd parity, 1 stop bit) handles bidirectional control: received commands adjust outputs, and a status message transmits automatically every 4 seconds.
- **Debounced, lockout-protected switch inputs** — Manual switches for light and fan use 10ms debounce and 2-second lockout timing to prevent bounce-induced false triggers.
- **Priority-based control logic** — Serial commands take precedence over manual switches, with defined timeout/override behavior when both are used together.
- **Closed-loop temperature regulation** — Heater/cooler activate automatically outside a 22–24°C deadband, with hysteresis to prevent output chattering from sensor noise.

## Validation

All timing-critical behavior was validated on real hardware using a Keysight oscilloscope, not just simulation:

| Behavior | Verified Result |
|---|---|
| Main loop LED toggle timing | Confirmed via oscilloscope capture of GPIO toggle rate |
| USART3 receiver (serial commands in) | Captured command bytes on the wire, confirmed correct decode |
| USART3 transmitter (status message out) | Captured transmitted status frame, confirmed correct encoding |
| 2-second key lockout | Measured directly — lockout window matched design spec |
| 10-second temperature/fan override lockout | Measured directly — matched design spec |
| Temperature formatting (mock ADC value) | Verified end-to-end: 5000 → correctly transmitted as "+50.00" |

Notably, cycle-counting in the Keil simulator gave misleading timing estimates (suggesting a ~1.93ms main loop, slower than the 1ms scheduler tick) — real hardware measurement showed actual execution in the microsecond range, confirming the simulator's peripheral timing isn't trustworthy and hardware validation was essential.

## I/O Mapping

| Function | Port/Pin | Type | Board Function |
|---|---|---|---|
| Temperature Sensor | PF10 | Analogue Input | Potentiometer |
| Light Switch | PA10 | Digital Input | Switch (SW4) |
| Light Intensity Sensor | PA8 | Digital Input | Switch (SW2) |
| Fan Switch | PB0 | Digital Input | Switch (SW5) |
| Heater Output | PF8 | Digital Output | LED 7 |
| Cooling Output | PB8 | Digital Output | LED 6 |
| Fan Control Output | PB1 | Digital Output | LED 5 |
| Light Control Output | PA9 | Digital Output | LED 2 |
| UART3 Rx / Tx | PB11 / PB10 | Alternate Function | — |

## Repository Contents

- `STM32F439_Template/src/main.c` — Main control loop, scheduler, receiver/transmitter, and control logic
- `STM32F439_Template/src/boardSupport.c`, `gpioControl.c` — Peripheral configuration (RCC, GPIO, USART3, ADC) and hardware abstraction
- `STM32F439_Template/inc/` — Header files (`config.h`, `ioMapping.h`, `main.h`, etc.) and vendor-supplied CMSIS/STM32 core headers
- `STM32F439_Template/STM32F439_Template.uvprojx` — Keil uVision project file
- `Design_Report.pdf` — Full write-up with configuration details, flowcharts, and oscilloscope validation

*Note: `inc/CMSIS/` and `RTE/Device/` contain ARM/STMicroelectronics vendor library files (not authored by the project team) — included so the project builds standalone in Keil.*

## Tools Used

Keil uVision (development and simulation), STM32F439 development board, Tera Term (serial terminal), Keysight oscilloscope (hardware validation).
