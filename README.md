# 🕊️ Angel FC – A Lightweight STM32 Flight Controller

**Angel FC** is a modular, RTOS-based flight controller firmware for drones and robotics, built around the STM32F411CEU6 microcontroller. Inspired by the Pixhawk architecture, Angel FC is designed with real-time performance, expandability, and code clarity in mind.

---

## ✈️ Features

- **Processor:** STM32F411CEU6 (ARM Cortex-M4 @ 100 MHz)
- **OS:** FreeRTOS (preemptive real-time kernel)
- **IMU:** MPU6050 (3-axis accelerometer + 3-axis gyroscope)
- **Barometer:** MPL3115A2 (altitude and pressure)
- **Magnetometer (Planned):** Future support for 3-axis magnetometers
- **Motor Outputs:** 6 PWM channels
- **RC Input:** PPM and SBUS
- **Indicators:** RGB LED (status feedback)
- **Data Logging:** Flash memory (internal/external)
- **Interfaces:**
  - 2x UART (serial ports)
  - I2C Bus
  - USB (CDC for telemetry, DFU for firmware update)
  - 3 ADC channels (3.3V)

---

## 🗂️ Project Structure

```plaintext
Angel-FC/
├── App
│   ├── Config
│   ├── Middleware
│   ├── Tasks
│   └── Utils
├── build
├── config
├── Core
│   ├── Inc
│   └── Src
├── Docs
├── Drivers
│   ├── CMSIS
│   ├── MyDrivers
│   └── STM32F4xx_HAL_Driver
├── Middlewares
│   ├── ST
│   └── Third_Party
└── USB_DEVICE
    ├── App
    └── Target
```

---

## 📦 Dependencies

- STM32CubeMX + STM32 HAL
- FreeRTOS
- CMSIS
- GCC / ARM Toolchain or STM32CubeIDE

---

## 🧠 Architecture Overview

- **Modular Tasks:** Sensor acquisition, control logic, and USB telemetry are separated into clean RTOS tasks.
- **Thread-Safe Data Sharing:** Sensor data accessed using mutex-protected shared memory.
- **Decoupled Drivers:** Hardware-specific drivers are cleanly abstracted from logic.
- **USB Telemetry:** Real-time sensor output and debugging over USB CDC.
- **Designed for Expansion:** Support for magnetometer, GPS, telemetry radio, SD logging, etc., planned.

---

## 🔧 Getting Started

1. Open `AngelFC.ioc` in STM32CubeMX and regenerate code.
2. Build the project using STM32CubeIDE or Make/GCC.
3. Flash via ST-Link or DFU.
4. Monitor sensor output via USB (CDC).

---

## 📈 Example Telemetry Output
```

[IMU] Gyro(deg/s) >> X:-1.07, Y:1.66, Z:0.99 | Acc(m/s2) >> X:0.62, Y:0.18, Z:9.13
```

---

## 📌 Roadmap

- [x] RTOS + task structure
- [x] MPU6050 IMU integration
- [x] MPL3115A2 barometer support
- [x] USB telemetry (CDC)
- [x] PWM motor output
- [ ] Magnetometer integration
- [ ] GPS & MAVLink over UART
- [ ] PID tuning interface
- [ ] Flash or SD card logging

---


## 📜 License

This project is licensed under the MIT License.

---

## 👤 Author

**Jesutofunmi Kupoluyi**  
Embedded Systems Engineer