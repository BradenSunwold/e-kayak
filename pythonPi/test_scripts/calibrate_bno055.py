"""
BNO055 calibration script.
Guides the user through calibrating the IMU, then saves offsets and radii
to a JSON file so they can be restored on future startups.

Calibration procedure:
  - Gyroscope:  Leave the sensor still for a few seconds.
  - Accelerometer: Place the sensor in 6 different stable positions (±X, ±Y, ±Z).
  - Magnetometer: Move the sensor in a figure-8 pattern in the air.
  - System: Reaches 3 once all three sub-sensors are calibrated.

Ctrl+C to abort without saving.
"""

import json
import time
from pathlib import Path
from adafruit_extended_bus import ExtendedI2C
import adafruit_bno055

CALIBRATION_FILE = Path(__file__).parent.parent / "config" / "bno055_calibration.json"

# Use bit-banged i2c-3 bus (SDA=GPIO17, SCL=GPIO27 per config.txt dtoverlay)
i2c = ExtendedI2C(3)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 Calibration")
print("=" * 50)
print("Move the sensor to calibrate each subsystem:")
print("  Gyroscope     — hold still")
print("  Accelerometer — rotate to 6 stable positions")
print("  Magnetometer  — figure-8 in the air")
print("=" * 50)
print("Waiting for all calibration levels to reach 3...\n")

try:
    while True:
        sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status
        print(f"\rCal  sys={sys_cal}  gyro={gyro_cal}  accel={accel_cal}  mag={mag_cal}  ", end="", flush=True)

        if sys_cal == 3 and gyro_cal == 3 and accel_cal == 3 and mag_cal == 3:
            print("\n\nFully calibrated!")
            break

        time.sleep(0.5)
except KeyboardInterrupt:
    print("\n\nAborted — calibration NOT saved.")
    raise SystemExit(1)

# Read offsets and radii from the sensor
calibration_data = {
    "offsets_accelerometer": list(sensor.offsets_accelerometer),
    "offsets_magnetometer": list(sensor.offsets_magnetometer),
    "offsets_gyroscope": list(sensor.offsets_gyroscope),
    "radius_accelerometer": sensor.radius_accelerometer,
    "radius_magnetometer": sensor.radius_magnetometer,
}

CALIBRATION_FILE.parent.mkdir(parents=True, exist_ok=True)
with open(CALIBRATION_FILE, "w") as f:
    json.dump(calibration_data, f, indent=2)

print(f"Calibration saved to {CALIBRATION_FILE}")
print(json.dumps(calibration_data, indent=2))
