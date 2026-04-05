"""
BNO055 IMU test script.
Reads Euler angles (heading/roll/pitch) over I2C and prints to console.
Ctrl+C to stop.
"""

import time
from adafruit_extended_bus import ExtendedI2C
import adafruit_bno055

# Use bit-banged i2c-3 bus (SDA=GPIO17, SCL=GPIO27 per config.txt dtoverlay)
i2c = ExtendedI2C(3)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 IMU Test — reading Euler angles (heading, roll, pitch)")
print("Waiting for calibration... move the sensor around.")
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
