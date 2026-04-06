"""
BNO055 IMU test script.
Reads Euler angles (heading/roll/pitch) over I2C and prints to console.
Loads saved calibration from config/bno055_calibration.json if available.
Ctrl+C to stop.
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

# Load saved calibration if available
if CALIBRATION_FILE.exists():
    with open(CALIBRATION_FILE, "r") as f:
        cal = json.load(f)
    sensor.offsets_accelerometer = tuple(cal["offsets_accelerometer"])
    sensor.offsets_magnetometer = tuple(cal["offsets_magnetometer"])
    sensor.offsets_gyroscope = tuple(cal["offsets_gyroscope"])
    sensor.radius_accelerometer = cal["radius_accelerometer"]
    sensor.radius_magnetometer = cal["radius_magnetometer"]
    print(f"Loaded calibration from {CALIBRATION_FILE}")
else:
    print("No saved calibration found — run calibrate_bno055.py first.")

print("BNO055 IMU Test — reading Euler angles (heading, roll, pitch)")
print("-" * 50)

while True:
    sys_cal, gyro_cal, accel_cal, mag_cal = sensor.calibration_status
    euler = sensor.euler
    if euler[0] is not None:
        heading, roll, pitch = euler
        print(f"Heading: {heading:7.2f}°  Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°  "
              f"Cal[sys={sys_cal} gyro={gyro_cal} accel={accel_cal} mag={mag_cal}]")
    else:
        print("Sensor not calibrated yet...")
    time.sleep(0.1)
