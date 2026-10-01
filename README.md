# H7 Embedded Control Code

## Overview

This repository contains the low-level embedded firmware for a robot control system built around the DaMiao MC02 development board based on the STM32H723VGT6 microcontroller. It implements the MCU-side control logic for chassis, gimbal, shooting, sensor acquisition, and communication with a high-level Jetson computer.

The code is designed for real-time control and coordination of robot subsystems, while exposing a UART interface for command and telemetry exchange with the high-level Jetson system.

## Key Features

- STM32H723VGT6 MCU firmware for DaMiao MC02
- FreeRTOS-based multitasking architecture
- Motor control via FDCAN for DJI-compatible and other actuator controllers
- Chassis, gimbal, shooting, and sensor feedback control loops
- Remote controller input processing
- UART / USB CDC communication with host / Jetson
- SEGGER SystemView support for runtime tracing and debugging
- Peripheral support for GPIO, ADC, SPI, timers, DMA, BDMA, FDCAN, and USB

## Project Structure

- `Application/Task/Inc`, `Application/Task/Src`
  - Application-level tasks and control logic
  - Control, CAN, INS, detection, and USB/PC communication tasks
- `BSP/Inc`, `BSP/Src`
  - Board support package and hardware peripheral wrappers
  - Drivers for ADC, CAN, DWT, GPIO, PWM, SPI, ticks, UART, and MCU init
- `Components/Device/Inc`, `Components/Device/Src`
  - Motor drivers, device abstractions, and communication helpers
  - Includes motor control modules and MiniPC/Jetson interface code
- `Components/Algorithm`, `Components/Controller`
  - Control algorithms, PID loops, state machines, and higher-level controllers
- `Core/Inc`, `Core/Src`
  - CubeMX-generated HAL setup, system startup, and FreeRTOS integration
- `Drivers/STM32H7xx_HAL_Driver`
  - STM32H7 HAL library sources
- `Middlewares/Third_Party/FreeRTOS`
  - FreeRTOS kernel and scheduling sources
- `SystemView/SEGGER`
  - SEGGER SystemView and RTT instrumentation libraries
- `USB_DEVICE`
  - USB device stack and USB CDC support
- `MDK-ARM`
  - Keil MDK project files and build artifacts

## Hardware Platform

- Target MCU: STM32H723VGT6
- Board: DaMiao MC02 development board
- Main high-level interface: UART or USB CDC to Jetson
- Motor control: FDCAN bus to drive DJI, Cubemars, and custom motors
- Sensor and actuator interfaces: ADC, SPI, GPIO, PWM, timers

## Firmware Architecture

- `main.c` initializes the MCU, caches, clocks, peripherals, and starts the FreeRTOS scheduler
- `Control_Task.c` implements the main robot control loop:
  - updates the state machine
  - reads sensor and remote control data
  - computes target and measured values
  - runs PID-based motion control for chassis, gimbal, and shooting
- Communication modules bridge the MCU with the Jetson/high-level host
- Device modules handle low-level CAN, UART, and USB messaging

## Jetson / High-Level Communication

- The MCU provides serial communication channels for command, status, and telemetry
- `USB_MiniPC_Task` supports a USB CDC connection as a virtual COM port
- UART-based protocols send robot state and receive motion commands
- This repository is intended to be paired with high-level planning and perception software on a Jetson system

## Development Environment

- IDE: Keil MDK-ARM 5.38
- STM32CubeMX: 6.12.0
- Compiler: Arm Compiler 6.19
- Host OS: Windows 11
- Optional editor: Visual Studio Code

## Notes

- This repository focuses on low-level firmware and hardware interfacing.
- High-level algorithms such as navigation and perception are expected to run on an external Jetson platform.
- Legacy documentation is included in `README.pdf`.
- `MDK-ARM` contains the Keil project files for building the firmware.

## Getting Started

### Opening the Project
Open `MDK-ARM/COD_H7_Template.uvprojx` in Keil µVision (look for the file marked with the green Keil icon).

### Understanding the Codebase

**1. Robot Configuration** — Start here  
Check `Components/controller/Robot_config.h` to identify which robot or subsystem is currently selected. This file is the entry point for understanding what the build is targeting.

**2. Control Logic**  
Explore the rest of `Components/controller/` for the core control logic, including:
- The **state machine** governing high-level behavior
- **Subsystem controllers** for each actuated or sensed component

**3. Device Drivers**  
Browse `Components/device/` to see how hardware is integrated — including the IMU, motors, and any other peripherals.

**4. Application Layer**  
Finally, look through:
- `Application/Task/` — FreeRTOS task definitions and scheduling
- `Application/User/Core/` — top-level initializations and high-level application logic


## Credits & License

This project is based on [COD-H7](https://github.com/GrassFanWang/COD-H7-Template) by GrassFan_Wang, licensed under the MIT License.
Our modifications are also released under the MIT License. See [LICENSE](LICENSE).



