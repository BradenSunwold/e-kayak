"""
BNO055 IMU test script.
Reads Euler angles (heading/roll/pitch) over I2C and prints to console.
Ctrl+C to stop.
"""

import time
import board
import adafruit_bno055

i2c = board.I2C()
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("BNO055 IMU Test — reading Euler angles (heading, roll, pitch)")
print("Waiting for calibration... move the sensor around.")
print("-" * 50)

while True:
    euler = sensor.euler
    if euler[0] is not None:
        heading, roll, pitch = euler
        print(f"Heading: {heading:7.2f}°  Roll: {roll:7.2f}°  Pitch: {pitch:7.2f}°")
    else:
        print("Sensor not calibrated yet...")
    time.sleep(0.1)
