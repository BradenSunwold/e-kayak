# E-Kayak — Oar Remote Controller

## Project Overview

Arduino-based remote controller for an electric kayak system, running on an Adafruit Feather M0 (ATSAMD21 Cortex-M0). Uses FreeRTOS_SAMD21 for real-time task scheduling. The oar controller reads IMU data from a BNO08x, transmits commands and sensor data to the kayak-side Raspberry Pi over an nRF24 RF link, and provides user feedback through a 12-LED NeoPixel ring.

## Hardware

- **MCU:** Adafruit Feather M0 — ATSAMD21 Cortex-M0
- **IMU:** BNO08x 9-axis IMU (I2C, 400 kHz) — stabilized rotation vector, accelerometer, gyroscope
- **RF Transceiver:** nRF24L01 — 2.4 GHz link to the Raspberry Pi kayak side (SPI, 4 MHz clock)
- **Display:** 12-LED WS2812B NeoPixel ring (GRB color format)
- **Input:** Physical button (pin 12) — single and double-click detection
- **Power:** Battery voltage read via analog divider on A0

## Development Environment

- **IDE:** Arduino IDE
- **RTOS:** FreeRTOS_SAMD21
- **Language:** C++ (Arduino framework)

## Architecture

Five main FreeRTOS tasks:

1. **ReadImuTask** — Event-driven (interrupt + 1 s watchdog). Reads BNO08x reports and converts quaternions to Euler angles.
2. **ButtonInputTask** (20 Hz) — Debounces button input, detects single/double clicks, controls speed level (0–3) and auto-mode toggle.
3. **ProcessOutputsTask** (50 Hz) — Central hub: processes fault flags, communication loss, battery calculations, and drives LED animation state.
4. **RfRadioTask** (40 Hz) — Transmits IMU + command data in two RF packets; receives kayak feedback (speed, battery, faults). 3 s communication timeout watchdog.
5. **LedPixelUpdaterTask** (25 Hz) — Renders NeoPixel animations (pulse, circular, gauge, startup sequences) from a frame queue.

Tasks communicate via FreeRTOS queues (6 total) and semaphores.

## RF Protocol

- **TX pipe:** `"1Node"` / **RX pipe:** `"2Node"`
- **Data rate:** RF24_2MBPS / **Power:** RF24_PA_LOW
- **Max payload:** 32 bytes
- Dynamic payloads enabled; 4 retries
- **Mode-based single packet:** Each TX cycle sends one self-describing packet. The mode byte (`fMode`, uint8 `MotorMode_t`) tells the Pi how to parse the payload:
  - **Manual** (`RfManualMsg_t`, 15 bytes): `[index, mode=0, speed, roll, pitch, yaw]`
  - **Auto** (`RfAutoMsg_t`, 27 bytes): `[index, mode=1, speed, accelX, gyroX, accelY, gyroY, accelZ, gyroZ]`
  - **Training** (reuses `RfAutoMsg_t`, 27 bytes): `[index, mode=2, speed=0, accelX, gyroX, accelY, gyroY, accelZ, gyroZ]` — automatically entered when manual mode + speed=0

## Important Constraints

- **RF message structs are a shared protocol** — `RfManualMsg_t`, `RfAutoMsg_t`, `PaddleCmdMsg_t`, and `KayakFeedbackMsg_t` must stay in sync with the Raspberry Pi side (`pythonPi` project). Changes here require matching updates there.
- **SAMD21 resource constraints** — limited RAM; task stack sizes are tightly tuned. Monitor free stack when adding functionality.
- **NeoPixel timing is sensitive** — LED updates must not be interrupted. The LedPixelUpdaterTask runs at lower priority to avoid starving higher-priority tasks, but its `show()` call briefly disables interrupts.

## Pin Assignments

| Function | Pin |
|---|---|
| RF24 CE | 6 |
| RF24 CSN | 5 |
| BNO08x CS | 10 |
| BNO08x INT | 18 |
| BNO08x RESET | 15 |
| Button | 12 |
| Battery voltage | A0 |
| Debug LED | 13 |

## Key Files

- `oarController.ino` — Main application: all task implementations and animation functions
- `TaskMetaData.hpp/.cpp` — Per-task timing wrapper (call rate, execution time, statistics)
- `TimingStats.hpp/.cpp` — Statistical analysis (min, max, avg, variance, std dev)
- `MetaData.hpp/.cpp` — Task call-rate tracking
- `ExecutionTimer.hpp/.cpp` — Task execution time measurement
- `LedPixelMaps.hpp/.cpp` — LED ring animation framework and frame generators
