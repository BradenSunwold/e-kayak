# E-Kayak — Raspberry Pi 5 Kayak Controller

## Project Overview

Python-based controller running on a Raspberry Pi 5. This is the main kayak-side component of an electric kayak system. It manages motor control, RF communication with a remote oar, IMU data logging, and will eventually incorporate ML-based paddle detection and autonomous fin steering.

## Hardware

- **Platform:** Raspberry Pi 5
- **RF Transceiver:** nRF24L01 — communicates with the oar remote over an RF link
- **Motor Controller:** BLDC motor controller — commanded over UART
- **Oar IMU:** Receives IMU data from the oar over the RF link
- **Future:** Onboard IMU and GPS for heading hold and training data auto-tagging

## Interfaces

- **nRF24 (RF link):** Receives mode-based single packets from the oar remote. Manual mode packets contain roll/pitch/yaw; auto mode packets contain raw accel/gyro. The mode byte in each packet determines the parse format.
- **UART:** Sends speed commands to the BLDC motor controller; monitors for motor faults

## Current Functionality

- Receives input commands from the oar remote over RF
- Commands the BLDC motor controller with speed setpoints over UART
- Monitors the motor controller for faults
- Logs all IMU data received from the oar

## Planned / In-Progress

- **ML paddle detection:** A machine learning algorithm will consume logged oar IMU data to infer when the user is paddling and which direction they intend to steer
- **Fin servo control:** Use ML steering intent to drive a servo on the kayak fin for directional assist
- **Onboard IMU:** For heading hold and complementing oar IMU data
- **GPS:** For auto-tagging training data and maintaining a straight heading

## Important Constraints

- **`StatusType` in `KayakDefines.py` is a shared protocol struct** — it defines what is transmitted from the Pi to the oar remote controller. Any changes to `StatusType` require a matching update in the oar controller project. The oar side only needs to know there is a fault so it can flash red and stop commanding.
- **RF RX parsing is mode-based** — `RfManager.py` reads the mode byte (uint8) from each incoming packet. Three modes defined in `MotorMode` enum (`KayakDefines.py`): MANUAL=0 (`BBBfff`, 16 bytes, roll/pitch/yaw), AUTO=1 (`BBBffffff`, 28 bytes, accel/gyro), TRAINING=2 (same 28-byte format as auto). Training mode is automatically entered by the oar when in manual mode with speed=0; motor is off and fin is centered. Changes to oar-side structs require matching updates to the format strings here.

## Key Files

- `main.py` — Entry point
- `BoatManager.py` — Top-level boat state management
- `MotorManager.py` — BLDC motor controller interface and fault monitoring
- `RfManager.py` / `RF.py` — nRF24 RF link management
- `KayakDefines.py` — Shared constants and definitions
- `FIR.py` — FIR filter implementation (likely for IMU data)
- `config/` — Configuration files
- `logs/` — IMU and operational logs
