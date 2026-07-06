# E-Kayak — Raspberry Pi 5 Kayak Controller

## Project Overview

Python-based controller running on a Raspberry Pi 5. This is the main kayak-side component of an electric kayak system. It manages motor control, RF communication with a remote oar, IMU logging, fin servo steering, and ML-based paddle-assist inference.

## Hardware

- **Platform:** Raspberry Pi 5
- **RF Transceiver:** nRF24L01 — communicates with the oar remote over an RF link
- **Motor Controller:** VESC BLDC motor controller — commanded over UART, enabled via GPIO
- **Oar IMU:** Raw accel/gyro streamed from the oar over the RF link (20 Hz)
- **Onboard IMU:** BNO055 on I2C bus 3 — kayak heading (fused) plus raw accel/gyro for ML labels
- **Fin Servo:** Steering fin on GPIO 12, driven via gpiozero + pigpio
- **Future:** GPS for auto-tagging training data and maintaining a straight heading

## Interfaces

- **nRF24 (RF link):** Receives mode-based single packets from the oar remote. Manual mode packets contain roll/pitch/yaw; auto mode packets contain raw accel/gyro. The mode byte in each packet determines the parse format.
- **UART:** Sends speed commands to the VESC; monitors for motor faults
- **InfluxDB:** All managers write telemetry points for Grafana dashboards. Fields only — never tag points with per-sample values (class, mode); varying tags split every field into separate series and break the plots.

## Current Functionality

- Receives input commands from the oar remote over RF
- Commands the VESC with speed setpoints over UART; monitors faults with a latching/recovering fault state machine
- **Motor modes** (`MotorMode` enum): MANUAL (oar speed setting → RPM), TRAINING (motor off, fin centered — clean data collection), AUTO (ML assist drives RPM)
- **ML assist (AUTO mode):** `MlManager` runs the assist regression ONNX model in its own OS process (CPU-pinned, batch-of-1 at 20 Hz) over oar IMU windows. The raw prediction is queued to `MotorManager`, which maps it to RPM:
  `assistCommand = clamp(prediction × assistGain, 0, 1)` → `targetRpm = deadbandRpm + assistCommand × assistMaxRpm`, smoothed by an asymmetric RC filter (ramp-up tau when the target is above the current output, ramp-down tau below — the "sustain vs pulse" decay knob)
- **Fin steering (MANUAL mode):** oar pitch beyond a deadband gives open-loop proportional fin control; inside the deadband the current heading is latched and a PID holds it. TRAINING/AUTO center the fin (ML turn model integration is future work)
- Logs all IMU data (oar + kayak) for offline training in `mlTraining/`

## ML Artifact Pipeline

Trained in `mlTraining/` on the Mac, consumed here via ONNX (never torch on the Pi):

1. `train.py --model assist` → `checkpoints/assist_best.pt` + `assist_norm_stats.json` + `assist_meta.json`
2. `export_onnx.py --model assist` → `assist_best.onnx` + `assist_onnx_meta.json`
3. `config.yaml` (`mlManager:` section) points at the three runtime artifacts: `assistModelPath`, `assistModelMetaPath`, `assistNormStatsPath`

The ONNX meta sidecar carries `window_size`, channel info, and `label_norm_scale` so training-side constants are never duplicated on the Pi. The model output is a raw regression scalar in normalized label units (~[-1, 1]; 1.0 ≈ 95th-percentile stroke intensity, negative = decelerating); multiply by `label_norm_scale` for physical units (m/s² of future kayak forward acceleration). Predictions are z-scored with the *training-time* channel stats from the norm-stats file — never statistics computed from live data. Smoke-test artifacts with `test_scripts/verify_onnx.py --model assist`.

A parallel turn model (`--model turn`) exists in training but is not yet wired into the Pi; when it lands it gets its own queue alongside `assistToMotorQueue`.

## Important Constraints

- **`StatusType` in `KayakDefines.py` is a shared protocol struct** — it defines what is transmitted from the Pi to the oar remote controller. Any changes to `StatusType` require a matching update in the oar controller project. The oar side only needs to know there is a fault so it can flash red and stop commanding.
- **RF RX parsing is mode-based** — `RfManager.py` reads the mode byte (uint8) from each incoming packet. Three modes defined in `MotorMode` enum (`KayakDefines.py`): MANUAL=0 (`BBBfff`, 16 bytes, roll/pitch/yaw), AUTO=1 (`BBBffffff`, 28 bytes, accel/gyro), TRAINING=2 (same 28-byte format as auto). Training mode is automatically entered by the oar when in manual mode with speed=0; motor is off and fin is centered. Changes to oar-side structs require matching updates to the format strings here.
- **`MlManager` is a `multiprocessing.Process`, not a thread** — it must only be handed picklable config in `__init__`; logger, Influx writer, and ONNX session are constructed inside `run()` in the child process.
- **Training-mode data purity** — the ML models train only on TRAINING-mode samples (motor off), because motor thrust contaminates the kayak IMU labels. Don't change TRAINING mode's motor/fin behavior without considering the training pipeline.

## Key Files

- `main.py` — Entry point
- `BoatManager.py` — Top-level wiring: config load, loggers, queues between managers, process/thread lifecycle
- `MotorManager.py` — VESC interface, fault state machine, motor-mode RPM logic (incl. AUTO assist mapping), fin servo + heading PID
- `MlManager.py` — Assist regression ONNX inference in a separate process
- `RfManager.py` / `RF.py` — nRF24 RF link management
- `ImuManager.py` — Onboard BNO055 (heading to motor, raw samples to logs)
- `InfluxWriter.py` — Batched InfluxDB telemetry writer
- `RCFilter.py` / `PIDController.py` — Control primitives (FIR.py is orphaned, no longer imported)
- `KayakDefines.py` — Shared constants and protocol definitions
- `config/config.yaml` — All runtime configuration
- `test_scripts/` — Hardware/bench checks (servo sweep, BNO055 calibration, ONNX verification)
- `logs/` — IMU and operational logs (the training corpus)
