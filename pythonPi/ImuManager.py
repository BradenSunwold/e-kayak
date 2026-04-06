import json
import time
import struct
import sched
import threading
import queue
from pathlib import Path
from adafruit_extended_bus import ExtendedI2C
import adafruit_bno055


class ImuManager(threading.Thread):
    def __init__(self, configDictionary, logger, outgoingQueue, influxWriter=None):
        super().__init__()
        self.mLogger = logger
        self.mInfluxWriter = influxWriter
        self.mLogger.info('**** IMU Manager Starting Up *****')
        self.mLogger.info('*************************************')

        # Queue to send heading data to motor thread
        self.mOutgoingQueue = outgoingQueue

        # Parse configs
        self.mConfigurator = configDictionary
        self.mI2cBus = self.mConfigurator['i2cBus']
        self.mCalibrationFile = self.mConfigurator['calibrationFilePath']

        # Set up scheduler
        self.mScheduler = sched.scheduler(time.time, time.sleep)
        self.mReadInterval = self.mConfigurator['imuReadRateInMilliseconds'] / 1000
        self.mNextReadTime = 0

        # Shutdown flag
        self.mShutdownRequested = False

        # Initialize BNO055 over I2C
        i2c = ExtendedI2C(self.mI2cBus)
        self.mSensor = adafruit_bno055.BNO055_I2C(i2c)

        # Load saved calibration if available
        self._LoadCalibration()

    def _LoadCalibration(self):
        """Load saved calibration offsets and radii from JSON file."""
        calPath = Path(self.mCalibrationFile)
        if calPath.exists():
            try:
                with open(calPath, 'r') as f:
                    cal = json.load(f)
                self.mSensor.offsets_accelerometer = tuple(cal['offsets_accelerometer'])
                self.mSensor.offsets_magnetometer = tuple(cal['offsets_magnetometer'])
                self.mSensor.offsets_gyroscope = tuple(cal['offsets_gyroscope'])
                self.mSensor.radius_accelerometer = cal['radius_accelerometer']
                self.mSensor.radius_magnetometer = cal['radius_magnetometer']
                self.mLogger.info('Loaded BNO055 calibration from %s', calPath)
            except Exception as e:
                self.mLogger.error('Failed to load calibration: %s', e)
        else:
            self.mLogger.warning('No calibration file found at %s', calPath)

    def ReadImu(self):
        if self.mShutdownRequested:
            return

        try:
            euler = self.mSensor.euler
            sys_cal, gyro_cal, accel_cal, mag_cal = self.mSensor.calibration_status

            if euler[0] is not None:
                heading, roll, pitch = euler

                # Pack heading as a float and send to motor thread
                payload = struct.pack('f', heading)
                self.mOutgoingQueue.put(payload)

                self.mLogger.debug('Kayak IMU  heading=%.2f  roll=%.2f  pitch=%.2f  '
                                   'cal[sys=%d gyro=%d accel=%d mag=%d]',
                                   heading, roll, pitch,
                                   sys_cal, gyro_cal, accel_cal, mag_cal)
            else:
                self.mLogger.debug('Kayak IMU not ready yet')

        except Exception as e:
            self.mLogger.error('ReadImu exception: %s', e)

        self.mNextReadTime += self.mReadInterval
        self.mScheduler.enterabs(self.mNextReadTime, 1, self.ReadImu)

    def StartScheduler(self):
        now = time.time()
        self.mNextReadTime = now

        self.mScheduler.enterabs(self.mNextReadTime, 1, self.ReadImu)
        self.mScheduler.run()

    def shutdown(self):
        """Signal the IMU manager to stop."""
        self.mLogger.info('IMU shutdown requested')
        self.mShutdownRequested = True

    def run(self):
        self.StartScheduler()
